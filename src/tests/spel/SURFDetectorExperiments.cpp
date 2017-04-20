// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

#include <gtest/gtest.h>
#include <detector.hpp>
#include <surfDetector.hpp>
#include "limbLabel.hpp"
#include "spelParameters.hpp"
#include "TestsFunctions.hpp"
#include "spelGeometry.hpp"

using namespace std;
#if defined (HAVE_OPENCV_XFEATURES2D)
using namespace xfeatures2d;
#endif

namespace SPEL
{
 struct Parameters
  {
    float minHessian = 300;
    float searchDistCoeff = 0.5f;
    float minTheta = 90.0f;
    float maxTheta = 100.0f;
    float stepTheta = 10.0f;
    float uniqueLocationCandidates = 0.1f;
    float uniqueAngleCandidates = 0.1f;

    float isWeakThreshold = 0.1f;
    float searchStepCoeff = 0.2f;    
  };

 ostream& operator<<(ostream& os, Parameters &P)
 {
   os << "minHessian = " << P.minHessian << endl;
   os << "searchDistCoeff = " << P.searchDistCoeff << endl;
   os << "minTheta = " << P.minTheta << endl;
   os << "maxTheta = " << P.maxTheta << endl;
   os << "stepTheta = " << P.stepTheta << endl;
   os << "uniqueLocationCandidates = " << P.uniqueLocationCandidates << endl;
   os << "uniqueAngleCandidates = " << P.uniqueAngleCandidates << endl;
   os << "isWeakThreshold = " << P.isWeakThreshold << endl;
   os << "searchStepCoeff = " << P.searchStepCoeff << endl;
   return os;
 }
  Parameters SetDetectParameters(map<string, float> params)
  {
    params.emplace(COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN());
    params.emplace(COMMON_SURF_DETECTOR_PARAMETERS::KNN_MATCH_COEFFICIENT());
    params.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT());
    params.emplace(DETECTOR_DETECT_PARAMETERS::MIN_THETA());
    params.emplace(DETECTOR_DETECT_PARAMETERS::MAX_THETA());
    params.emplace(DETECTOR_DETECT_PARAMETERS::STEP_THETA());
    params.emplace(
       DETECTOR_DETECT_PARAMETERS::UNIQUE_LOCATION_CANDIDATES_COEFFICIENT());
    params.emplace(
       DETECTOR_DETECT_PARAMETERS::UNIQUE_ANGLE_CANDIDATES_COEFFICIENT());
    params.emplace(DETECTOR_DETECT_PARAMETERS::SCALE_COEFFICIENT());
    params.emplace(
       DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_MULT_COEFFICIENT());

    params.emplace(DETECTOR_DETECT_PARAMETERS::ROTATION_THRESHOLD());
    params.emplace(DETECTOR_DETECT_PARAMETERS::IS_WEAK_THRESHOLD());
    params.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT());

    Parameters P;

    P.minHessian = params.at(
       COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN().first);
    P.searchDistCoeff = params.at(
       DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT().first);
    P.minTheta = params.at(DETECTOR_DETECT_PARAMETERS::MIN_THETA().first);
    P.maxTheta = params.at(DETECTOR_DETECT_PARAMETERS::MAX_THETA().first);
    P.stepTheta = params.at(DETECTOR_DETECT_PARAMETERS::STEP_THETA().first);
    P.uniqueLocationCandidates = params.at(
       DETECTOR_DETECT_PARAMETERS::UNIQUE_LOCATION_CANDIDATES_COEFFICIENT().first);
    P.uniqueAngleCandidates = params.at(
       DETECTOR_DETECT_PARAMETERS::UNIQUE_ANGLE_CANDIDATES_COEFFICIENT().first);
    P.isWeakThreshold = params.at(
       DETECTOR_DETECT_PARAMETERS::IS_WEAK_THRESHOLD().first);
    P.searchStepCoeff = params.at(
       DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT().first);

    return P;
  }  

  struct SkeletonModel
  {
    vector<cv::KeyPoint> SkeletonKeypoints;
    cv::Mat SkeletonDescriptors;
    map<int, int> PartsKeypointsCount;
    //vector<cv::KeyPoint> BackgroundKeypoints;
    //cv::Mat BackgroundDescriptors;
  };

  // Sorting parts by the polygons area, not optimal, for testing only
  vector<int> PolygonsPriority(map<int, spelRECT<Point2f>> PartRects)
  {
    vector<float> PolygonsArea(PartRects.size());
    for (int i = 0; i < PartRects.size(); i++)
    {
      vector<Point2f> temp = PartRects[i].asVector();
      float PartLenght = BodyPart::getBoneLength(Point2f(0, 0), temp[1] - temp[0]);
      float PartWidth = BodyPart::getBoneLength(Point2f(0, 0), temp[3] - temp[0]);
      PolygonsArea[i] = PartLenght*PartWidth;
      temp.clear();
    }

    vector<int> SortedIndexes;
    const float imageArea = FLT_MAX;
    for (int i = 0; i < PolygonsArea.size(); i++)
    {
      int t = -1;
      float minArea = imageArea;
      for (int k = 0; k < PolygonsArea.size(); k++)
        if(PolygonsArea[k] < minArea)
        {
          minArea = PolygonsArea[k];
          t = k;
        }

      PolygonsArea[t] = imageArea;
      SortedIndexes.push_back(t);
    }

    PolygonsArea.clear();
    return SortedIndexes;
  }

  void SingleFrameTrain(Frame* frame, SkeletonModel &X, Parameters P, bool UseOnlyMaskKeypoints = true, bool usePartAreaPriority = true)
  {
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SurfFeatureDetector::create(P.minHessian);

    //if (frame->getFrametype() == KEYFRAME)

    std::vector<cv::KeyPoint> FrameKeypoints;

    cv::Mat image = frame->getImage();
    cv::Mat Mask = frame->getMask();
    detector->detect(image, FrameKeypoints);

    //imwrite("SurfDetector_mask.bmp", Mask);
    //cout << "FrameKeypoints[0].class_id = " << FrameKeypoints[0].class_id << endl;

    vector<KeyPoint> MaskKeypoins;
    if (UseOnlyMaskKeypoints)
    {
      for(unsigned int p = 0; p < FrameKeypoints.size(); p++)
        if(Mask.at<uchar>(FrameKeypoints[p].pt) < 10)
        {
          FrameKeypoints[p].class_id = -1;
          //X.BackgroundKeypoints.push_back(FrameKeypoints[p]);
        }
        else
        {
          FrameKeypoints[p].class_id = -2;
          MaskKeypoins.push_back(FrameKeypoints[p]);
        }
      }

    if (!UseOnlyMaskKeypoints)
      MaskKeypoins = FrameKeypoints;

    //cout << "FrameKeypoints size = " << FrameKeypoints.size() << endl;

    Skeleton skeleton = frame->getSkeleton();
    map<int, spelRECT<Point2f>> PartRects = SkeletonRects(skeleton);

    // Sorting polygons by area
    vector<int> SortedIndexes;
    if(usePartAreaPriority)
      SortedIndexes = PolygonsPriority(PartRects);
    else
    {
      for (int i = 0; i < PartRects.size(); i++)
        SortedIndexes.push_back(i);
    }

    // Create part models
    if(X.PartsKeypointsCount.size() == 0)
      for (unsigned int k = 0; k < PartRects.size(); k++)
        X.PartsKeypointsCount.emplace(std::pair<int, int>(k, 0));

    for (unsigned int p = 0; p < MaskKeypoins.size(); p++)
      if(MaskKeypoins[p].class_id !=-1)
        for (unsigned int k = 0; k < SortedIndexes.size(); k++)
        {
          vector<Point2f> PartPolygon = PartRects[SortedIndexes[k]].asVector();
          if (pointPolygonTest(PartPolygon, MaskKeypoins[p].pt, false) > 0)
          {
            MaskKeypoins[p].class_id = SortedIndexes[k];
            X.PartsKeypointsCount.at(SortedIndexes[k])++;
            X.SkeletonKeypoints.push_back(MaskKeypoins[p]);
          }
        }

    cv::Mat FrameDescriptors;
    cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    extractor->compute(image, X.SkeletonKeypoints, X.SkeletonDescriptors);

    detector->clear();
    extractor->clear();
    SortedIndexes.clear();
  }

  void Train(vector<Frame*> frames, SkeletonModel &X, Parameters P, bool putMessages = true)
  {
    if (putMessages) DebugMessage(" SURFDetector experiments Train started", 2);

    // Training on all keyframes
    for(int i = 0; i < frames.size(); i++)
      if (frames[i]->getFrametype() == KEYFRAME)
      {
        SingleFrameTrain(frames[i], X, P);
        if (putMessages)
        {
          stringstream s;
          s << "Training, KeyframeID = " << i;
          DebugMessage(s.str(), 2);
          s.clear();
        }
      }

    // Put debug messages
    //vector<int> EmptyParts; // Search empty part models
    if (putMessages)
      for (int i = 0; i < X.PartsKeypointsCount.size(); i++)
      {
        if (X.PartsKeypointsCount.at(i) == 0)
        {
          //EmptyParts.push_back(i);
          stringstream s;
          s << " Body Part " << i << ": keypoints not found, part model is empty";
          DebugMessage(s.str(), 2);
          s.clear();
        }
      }
      
    if (putMessages) DebugMessage(" SURFDetector experiments Train completed", 2);
  }

  class compareLabels
  {
  public:
    bool operator () (LimbLabel X, LimbLabel Y)
    {
      return (X.getAvgScore() > Y.getAvgScore());
    }
  };

  map<uint32_t, vector<LimbLabel>> Detect(Frame* frame, SkeletonModel &Trained, Parameters P, bool PutMessages = false, bool UseOnlyMaskKeypoints = true, bool CheckMatches = false, int useBadMatches = false, bool useMulct = false)
  {
    if (PutMessages) DebugMessage(" SURFDetector experiments Detect started", 2);
    cv::Mat image = frame->getImage();
    cv::Mat Mask = frame->getMask();
    //imwrite("SurfDetector_mask.bmp", Mask);

    // Calculate frame keypoints
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SurfFeatureDetector::create(P.minHessian);
    std::vector<cv::KeyPoint> FrameKeypoints;
    detector->detect(image, FrameKeypoints);

    // Select mask keypoints
    vector<KeyPoint> MaskKeypoins;
    if (UseOnlyMaskKeypoints)
    {
      for (unsigned int p = 0; p < FrameKeypoints.size(); p++)
        if (Mask.at<uchar>(FrameKeypoints[p].pt) < 10)
          FrameKeypoints[p].class_id = -1;
        else
        {
          FrameKeypoints[p].class_id = -2;
          MaskKeypoins.push_back(FrameKeypoints[p]);
        }
    }
    if (!UseOnlyMaskKeypoints)
      MaskKeypoins = FrameKeypoints;

    if (PutMessages)
    {
      stringstream s;
      s << " Mask keypoints count: " << MaskKeypoins.size();
      DebugMessage(s.str(), 2);
      s.clear();
    }

    // Extract descriptor
    cv::Mat FrameDescriptors;
    cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    extractor->compute(image, MaskKeypoins, FrameDescriptors);

    // Create skeleton model foe the current frame
    SkeletonModel Local;
    Local.SkeletonKeypoints = MaskKeypoins;
    Local.SkeletonDescriptors = FrameDescriptors;

    // Create matches
    int n = 1; // Trained.SkeletonKeypoints.size();
    if (CheckMatches) n = 2;
    if (useBadMatches)
    {
      useMulct = false;
      CheckMatches = false;
      n = Trained.SkeletonKeypoints.size();
    }

    // Put debug info
    /*cout << "UseOnlyMaskKeypoints = " << UseOnlyMaskKeypoints << endl;
    cout << endl << "CheckMatches = " << CheckMatches << endl;
    cout << "useBadMatches = " << useBadMatches << endl;
    cout << "useMulct = " << useMulct << endl;
    cout << "MatchesCount = " << n << endl << endl;*/

    cv::FlannBasedMatcher matcher;
    vector<vector<DMatch>> matches;
    matcher.knnMatch(FrameDescriptors, Trained.SkeletonDescriptors, matches, n);
    if (PutMessages) DebugMessage(" Matches was created", n);

    // Search maximal matches distance
    float maxDist = 0.0f;
    for(int i = 0; i < matches.size(); i++)
      for(int k = 0; k < n; k++)
        if (matches[i][k].distance > maxDist)
          maxDist = matches[i][k].distance;

    // Normalize and inverse matches distances
    for(int i = 0; i < matches.size(); i++)
      for(int k = 0; k < n; k++)
        matches[i][k].distance = 1.0f - matches[i][k].distance/maxDist;
    if (PutMessages) DebugMessage(" Matches was normalized", 2);

    Skeleton skeleton = frame->getSkeleton();
    map<int, spelRECT<Point2f>> PartRects = SkeletonRects(skeleton);

    map<uint32_t, vector<LimbLabel>> Labels;

    for (int id = 0; id < PartRects.size(); id++)
    {
      if(PutMessages && Trained.PartsKeypointsCount.at(id) == 0)
      {
        stringstream s;
        s << " Body Part " << id << " model is empty";
        DebugMessage(s.str(), 2);
        s.clear();
      }
      if(Trained.PartsKeypointsCount.at(id) > 0) // or " > threshold"
      {
        vector<LimbLabel> PartLabels;

        vector<Point2f> PartPolygon = PartRects[id].asVector();
        Point2f dL = PartPolygon[1] - PartPolygon[0];
        Point2f dW = PartPolygon[3] - PartPolygon[0];
        float PartLenght = sqrt(dL.x*dL.x + dL.y*dL.y);
        float PartWidth = sqrt(dW.x*dW.x + dW.y*dW.y);
        vector<Point2f> LabelPolygon(4), RotatedPolygon(4);

        Point2f PartCenter = 0.5f*(PartPolygon[3] + PartPolygon[1]);
        float SearchDist = P.searchDistCoeff*PartLenght;
        float minStep = abs(P.searchStepCoeff*PartWidth);
        minStep = max(minStep, 2.0f);
        

        int LabelsPerPart = 0;

        //uint8_t b = 0, g = 0; // visualization color coefficients
        //cout << endl << "PartID = " << id << ": Center = " << PartCenter << ", Width = " << PartWidth <<", Lenght = " << PartLenght << ", Step = " << minStep << endl;

        float PartAngle = spelHelper::getAngle(PartPolygon[0], PartPolygon[1]);
        for (float angle = - P.minTheta; angle < P.maxTheta; angle += P.stepTheta)
        { 
          // Rotation of the part polygon
          float LabelAngle = PartAngle + angle;
          for(int t = 0; t < LabelPolygon.size(); t++)
            RotatedPolygon[t] = spelHelper::rotatePoint2D(PartPolygon[t], PartCenter, angle);
          //b++;
          for(float x = -SearchDist; x < SearchDist; x += minStep)
            for(float y = - SearchDist; y < SearchDist; y += minStep)
            {
              //g++;
              LabelsPerPart++;

              // Shift label polygon
              for (int t = 0; t < LabelPolygon.size(); t++)
                LabelPolygon[t] = RotatedPolygon[t] + Point2f(x, y);
              Point2f LabelCenter = PartCenter + Point2f(x, y);

              // Calculate score for current label
              float LabelScore = 0.0f;
              float LabelMulct = 0.0f;
              int N = 0;
              for (unsigned int p = 0; p < MaskKeypoins.size(); p++)
                if (pointPolygonTest(LabelPolygon, MaskKeypoins[p].pt, false) > 0)
                {
                  bool b = true;
                  if(CheckMatches)
                    b = 0.8*matches[p][0].distance > matches[p][1].distance;			
                  bool NoMatches = true;
                  for (int m = 0; m < n; m++)
                  {
                    if (Trained.SkeletonKeypoints[matches[p][m].trainIdx].class_id == id && b)
                    {
                      N++;
                      LabelScore = LabelScore + pow(10.0f, -m)*matches[p][m].distance;
                      NoMatches = false;
                    }
                  }
                  if (useMulct && NoMatches) LabelMulct = LabelMulct + matches[p][0].distance;
                }

              if (PutMessages && N == 0)
              {
                stringstream s;
                s << " A significant keypoints for current Limb Label not found: PartID = " << id <<", Center = " << LabelCenter << ", Angle = " << LabelAngle;
                DebugMessage(s.str(), 2);
                s.clear();
              }

              // Cleate LimbLabel
              if (useMulct)
                LabelScore = LabelScore / (LabelScore + LabelMulct);
              Score score(LabelScore, "");
              vector<Score> scores;
              scores.push_back(score);
              LimbLabel Label(id, LabelCenter, LabelAngle, LabelPolygon, scores);
              PartLabels.push_back(Label);
              scores.clear();

              //PutPartRect(image, LabelPolygon, Scalar(b, g, id * 10));
              //cout << LabelCenter << ", " << LabelAngle;
              //cout << ", " << LabelScore << endl;
            }
        }
        sort(PartLabels.begin(), PartLabels.end(), compareLabels());
        Labels.emplace(std::pair<int, std::vector<LimbLabel>>(id, PartLabels));
        PartLabels.clear();
      
        if (PutMessages)
        {
          stringstream s;
          s << " Limb Labels count per Body Part " << id <<": " << LabelsPerPart;
          DebugMessage(s.str(), 2);
          s.clear();
        }
        //cout << " Limb Labels count per Body Part: " << LabelsPerPart << endl;
      }
    }

    //imwrite("Detect.bmp", image);
    if (PutMessages) DebugMessage(" SURFDetector experiments Detect completed", 2);
    return Labels;
  }

  TEST(SURFDetectorExperiments, SingleFrameTrain)
  {
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/C/", "skier.xml");
    vector<Frame*> frames = project.getFrames();

    TestProjectLoader project_pattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Frames = project_pattern.getFrames();

    map<string, float> params;
    Parameters P = SetDetectParameters(params);

    SkeletonModel X;
    SingleFrameTrain(frames[0], X, P);
    SingleFrameTrain(frames[2], X, P);
    for (int i = 0; i < X.PartsKeypointsCount.size(); i++)
      cout <<"Part[ " << i << "] keypoints size = " << X.PartsKeypointsCount[i] << endl;

    cout << endl;

    SkeletonModel Y;
    Train(frames, Y, P);
    for (int i = 0; i < X.PartsKeypointsCount.size(); i++)
      cout << "Part[ " << i << "] keypoints size = " << Y.PartsKeypointsCount[i] << endl;

    // Detect
    /*frames[1]->setSkeleton(Frames[1]->getSkeleton());
    map<uint32_t, vector<LimbLabel>> Labels = Detect(frames[1], X, P);*/

    frames.clear();
    Frames.clear();
  }
  
  TEST(SURFDetectorExperiments, Detect)
  {
    //Load the input data
    //TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/A/", "trijumpSD_shortcut.xml");
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/C/", "skier.xml");
    vector<Frame*> SFrames = project.getFrames();

    TestProjectLoader project_pattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Frames = project_pattern.getFrames();
    Frame* Pattern = Frames[1];
    Skeleton SkeletonPattern = Pattern->getSkeleton(); // Copying the manually marked skeleton

    // Create actual values
    map<string, float> params;
    Parameters P = SetDetectParameters(params);

    SkeletonModel X;
    long train_t0 = clock();
    Train(SFrames, X, P);
    long train_t1 = clock();

    long detect_t0 = clock();
    SFrames[1]->setSkeleton(SkeletonPattern);
    map<uint32_t, vector<LimbLabel>> Labels = Detect(SFrames[1], X, P); //Run experimental detect
    long detect_t1 = clock();


    // Set test parameters
    int TopLabelsCount = 4; // Size of "labels top list"
    float TolerableLinearError = 30; // Linear error in pixels

    // Compare limb labels with ideal bodyparts from keyframe 
    map<int, vector<float>> LinearErrors = LabelsLinearErrors(Labels, SkeletonPattern);
    map<int, vector<float>> AngleErrors = LabelsAngleErrors(Labels, SkeletonPattern);

    // Write debug information into text file
    string OutputFileName = "DetectTest_SURFExperiments.txt";
    ofstream fout(OutputFileName);
    fout << "\n\ DETECT PARAMETERS\n\n";
    fout << P;
    fout << "\n  DETECT RESULTS\n\n";
    PutSigificantErrors(fout, LinearErrors, AngleErrors, TopLabelsCount);
    PutLabels(fout, "", Labels, LinearErrors, AngleErrors, TopLabelsCount);
    PutLabels(fout, "", Labels, LinearErrors, AngleErrors, 0);
    fout.close();

    // Cheking the values and put test result
    vector<int> notFoundedParts = selectNotFoundedParts(SkeletonPattern, LinearErrors, Labels, TolerableLinearError, TopLabelsCount);
    cout << "\n EXECUTION TIME\n" << "\nTrain: " << spelHelper::clock_to_ms(train_t1 - train_t0) << " ms\n";
    cout << "Detect: " << spelHelper::clock_to_ms(detect_t1 - detect_t0) << " ms\n";

    cout << "\n\ DETECT PARAMETERS\n\n";
    cout << P;
    cout << "\n TEST PARAMETERS\n\n";
    cout << "Image size = " << Pattern->getMask().size() << endl;
    cout << "TolerableLinearError = " << TolerableLinearError << endl;
    cout << "TopLabelsCount = " << TopLabelsCount << endl;

    cout << "\n DETECT RESULTS\n\n";
    PutSigificantErrors(cout, LinearErrors, AngleErrors, TopLabelsCount);

    int FoundedPartsCount = Labels.size() - notFoundedParts.size();
    int PartsCount = SkeletonPattern.getPartTree().size();
    EXPECT_EQ(PartsCount, FoundedPartsCount);

    if (notFoundedParts.size() > 0)
    {
      cout << "Body parts with id: " << notFoundedParts[0];
      for (unsigned int i = 1; i < notFoundedParts.size(); i++)
        cout << ", " << notFoundedParts[i];
      cout << " - does not have effective labels in the top of labels list." << endl;
    }

    cout << "\nLimbLabels saved in file: " << OutputFileName << endl;    

    SFrames.clear();
    Frames.clear();
  }

}