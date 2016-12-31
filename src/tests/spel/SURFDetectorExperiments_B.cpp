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
#include "surfDetector2.hpp"
#include "limbLabel.hpp"
#include "spelParameters.hpp"
#include "TestsFunctions.hpp"
#include "spelHelper.hpp"

using namespace std;
#if defined (HAVE_OPENCV_XFEATURES2D)
using namespace xfeatures2d;
#endif

namespace SPEL
{
 struct Parameters_
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

    Size CellsCount = Size(4, 6);
  };

 ostream& operator<<(ostream& os, Parameters_ &P)
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

   os << "partCellsCount = " << P.CellsCount << endl;

   return os;
 }


  Parameters_ SetDetectParameters_(map<string, float> params)
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

    Parameters_ P;

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

  struct SkeletonModel_
  {
    vector<cv::KeyPoint> SkeletonKeypoints;
    cv::Mat SkeletonDescriptors;
    map<int, int> PartsKeypointsCount;
    //vector<cv::KeyPoint> BackgroundKeypoints;
    //cv::Mat BackgroundDescriptors;
  };

  // Sorting parts by the polygons area, not optimal, for testing only
  vector<int> PolygonsPriority_(map<int, spelRECT<Point2f>> PartRects)
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

  class CompareLabels_
  {
  public:
    bool operator () (LimbLabel X, LimbLabel Y)
    {
      return (X.getAvgScore() > Y.getAvgScore());
    }
  };

  int PartCellIndex(Point2f pt,int PartID, vector<Point2f> polygon, Size CellsCount)
  {
    Point2f dL = polygon[1] - polygon[0];
    Point2f dW = polygon[3] - polygon[0];
    float l = sqrt(dL.x*dL.x + dL.y*dL.y) / static_cast<float>(CellsCount.height);
    float w = sqrt(dW.x*dW.x + dW.y*dW.y) / static_cast<float>(CellsCount.width);
    Point2f d = dW;
    float a = sqrt(d.x*d.x + d.y*d.y);
    d = pt - polygon[0];
    float b = sqrt(d.x*d.x + d.y*d.y);
    d = pt - polygon[3];
    float c = sqrt(d.x*d.x + d.y*d.y);
    float p = 0.5*(a + b + c);
    float dw = 2 * sqrt(p*(p - a)*(p - b)*(p - c)) / a;
    float dl = sqrt(c*c - dw*dw);
    int nw = trunc(dw / w);
    int nl = trunc(dl / l);
    int id  = PartID * CellsCount.width*CellsCount.height + CellsCount.width*nl + nw;
    return id;
  }

  void SingleFrameTrain_(Frame* frame, SkeletonModel_ &X, Parameters_ P)
  {
    // Create frame keypoints
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SurfFeatureDetector::create(P.minHessian);
    std::vector<cv::KeyPoint> FrameKeypoints;
    cv::Mat image = frame->getImage();
    cv::Mat Mask = frame->getMask();
    detector->detect(image, FrameKeypoints);

    // Select mask keypoints
    vector<KeyPoint> MaskKeypoins;
    for (unsigned int p = 0; p < FrameKeypoints.size(); p++)
      if (Mask.at<uchar>(FrameKeypoints[p].pt) < 10)
        FrameKeypoints[p].class_id = -1;
      else
      {
        FrameKeypoints[p].class_id = -2;
        MaskKeypoins.push_back(FrameKeypoints[p]);
      }

    // Calculate part rects and parts cell size proections
    Skeleton skeleton = frame->getSkeleton();
    map<int, spelRECT<Point2f>> PartRects = SkeletonRects(skeleton);

    // Sorting polygons by area
    vector<int> SortedIndexes;
    SortedIndexes = PolygonsPriority_(PartRects);

    // Create part models
    if (X.PartsKeypointsCount.size() == 0)
      for (unsigned int k = 0; k < PartRects.size(); k++)
        X.PartsKeypointsCount.emplace(std::pair<int, int>(k, 0));

    for (unsigned int p = 0; p < MaskKeypoins.size(); p++)
      if(MaskKeypoins[p].class_id !=-1)
        for (unsigned int k = 0; k < SortedIndexes.size(); k++)
        {
          int PartID = SortedIndexes[k];
          vector<Point2f> PartPolygon = PartRects[PartID].asVector();
          if (pointPolygonTest(PartPolygon, MaskKeypoins[p].pt, false) > 0)
          {
            Point2f pt = MaskKeypoins[p].pt;
            MaskKeypoins[p].class_id = PartCellIndex(pt, PartID, PartPolygon, P.CellsCount);
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

  void Train_(vector<Frame*> frames, SkeletonModel_ &X, Parameters_ P)
  {
    DebugMessage(" SURFDetector experiments Train_2 started", 2);

    // Training on all keyframes
    for(int i = 0; i < frames.size(); i++)
      if (frames[i]->getFrametype() == KEYFRAME)
        SingleFrameTrain_(frames[i], X, P);
 
    DebugMessage(" SURFDetector experiments Train_2 completed", 2);
  }

  map<uint32_t, vector<LimbLabel>> Detect_(Frame* frame, SkeletonModel_ &Trained, Parameters_ P)
  {
    bool useMulct = false;
    bool CheckMatches = false;

    DebugMessage(" SURFDetector experiments Detect_2 started", 2);
    cv::Mat image = frame->getImage();
    cv::Mat Mask = frame->getMask();
    //imwrite("SurfDetector_mask.bmp", Mask);

    // Calculate frame keypoints
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SurfFeatureDetector::create(P.minHessian);
    std::vector<cv::KeyPoint> FrameKeypoints;
    detector->detect(image, FrameKeypoints);

    // Select mask keypoints
    vector<KeyPoint> MaskKeypoins;
      for (unsigned int p = 0; p < FrameKeypoints.size(); p++)
        if (Mask.at<uchar>(FrameKeypoints[p].pt) < 10)
          FrameKeypoints[p].class_id = -1;
        else
        {
          FrameKeypoints[p].class_id = -2;
          MaskKeypoins.push_back(FrameKeypoints[p]);
        }

    stringstream s1;
    s1 << " Mask keypoints count: " << MaskKeypoins.size() << endl;
    DebugMessage(s1.str(), 2);
    s1.clear();

    // Extract descriptor
    cv::Mat FrameDescriptors;
    cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    extractor->compute(image, MaskKeypoins, FrameDescriptors);

    // Create skeleton model foe the current frame
    SkeletonModel_ Local;
    Local.SkeletonKeypoints = MaskKeypoins;
    Local.SkeletonDescriptors = FrameDescriptors;

    // Create matches
    int n = 2;
    cv::FlannBasedMatcher matcher;
    vector<vector<DMatch>> matches;
    matcher.knnMatch(FrameDescriptors, Trained.SkeletonDescriptors, matches, n);
    DebugMessage(" Matches was created", n);

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
    DebugMessage(" Matches was normalized", 2);

    Skeleton skeleton = frame->getSkeleton();
    map<int, spelRECT<Point2f>> PartRects = SkeletonRects(skeleton);

    map<uint32_t, vector<LimbLabel>> Labels;

    for (int id = 0; id < PartRects.size(); id++)
    {
      if(Trained.PartsKeypointsCount.at(id) == 0)
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
        float minStep = abs( P.searchStepCoeff*PartWidth);
        minStep = max(minStep, 2.0f);
        
        int LabelsPerPart = 0;

        //uint8_t b = 0, g = 0; // visualization color coefficients
        //cout << endl << "PartID = " << id << ": Center = " << PartCenter << ", Width = " << PartWidth <<", Lenght = " << PartLenght << ", 

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

              for (unsigned int p = 0; p < MaskKeypoins.size(); p++)
                if (pointPolygonTest(LabelPolygon, MaskKeypoins[p].pt, false) > 0)
                {
                  bool b = 0.8*matches[p][0].distance > matches[p][1].distance || !CheckMatches;
                  if(b)
                  {
                    int partCellID = PartCellIndex(MaskKeypoins[p].pt, id, LabelPolygon, P.CellsCount);
                    if(Trained.SkeletonKeypoints[matches[p][0].trainIdx].class_id == partCellID)
                      LabelScore = LabelScore + matches[p][0].distance;
                    /*if (n > 1)
                      if (Trained.SkeletonKeypoints[matches[p][1].trainIdx].class_id == partCellID)
                        LabelScore = LabelScore + 0.1f*matches[p][1].distance;*/
                  }
                  else
                    if(useMulct)
                      LabelMulct++;
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
        sort(PartLabels.begin(), PartLabels.end(), CompareLabels_());
        Labels.emplace(std::pair<int, std::vector<LimbLabel>>(id, PartLabels));
        PartLabels.clear();
      
        stringstream s;
        s << " Limb Labels count per Body Part " << id <<": " << LabelsPerPart << endl;
        DebugMessage(s.str(), 2);
        s.clear();

        //cout << " Limb Labels count per Body Part: " << LabelsPerPart << endl;
      }
    }

    //imwrite("Detect.bmp", image);
    DebugMessage(" SURFDetector experiments Detect_2 completed", 2);
    return Labels;
  }
  
  TEST(SURFDetectorExperiments_B, Detect)
  {
    //Load the input data
    //TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/A/", "trijumpSD_shortcut.xml");
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/C/", "skier.xml");
    vector<Frame*> SFrames = project.getFrames();
    //for (int i = 0; i < SFrames.size(); i++)
    //  SFrames[i]->Resize(1024);
    //Frame* Pattern = SFrames[1];

    TestProjectLoader project_pattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Frames = project_pattern.getFrames();
    Frame* Pattern = Frames[1];
    Skeleton SkeletonPattern = Pattern->getSkeleton(); //Copying the manually marked skeleton

    // Create parameters
    map<string, float> params;
    Parameters_ P = SetDetectParameters_(params);
    
    // Create actual values
    SkeletonModel_ X;
    long train_t0 = clock();
    Train_(SFrames, X, P); //Run train  
    long train_t1 = clock();

    SFrames[1]->setSkeleton(SkeletonPattern); //Copy skeleton from pattern keyframe to current lockframe
    long detect_t0 = clock();
    map<uint32_t, vector<LimbLabel>> Labels = Detect_(SFrames[1], X, P); //Run detect
    long detect_t1 = clock();

    // Set test parameters
    int TopLabelsCount = 4; //Size of "labels top list"
    float TolerableLinearError = 30; //Linear error in pixels

    // Compare limb labels with ideal bodyparts from keyframe 
    map<int, vector<float>> LinearErrors = LabelsLinearErrors(Labels, SkeletonPattern);
    map<int, vector<float>> AngleErrors = LabelsAngleErrors(Labels, SkeletonPattern);

    // Write debug information into text file
    string OutputFileName = "DetectTest_SURFExperiments_B.txt";
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
    cout << "\n EXECUTION TIME\n" << "\nTrain: " << clock_to_ms(train_t1 - train_t0) << " ms\n";
    cout << "Detect: " << clock_to_ms(detect_t1 - detect_t0) << " ms\n";

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
    cout << "\nLimbLabels saved in file: " << OutputFileName << endl << endl;

    SFrames.clear();
    Frames.clear();
  }
  
  TEST(SURFDetectorExperiments_B, SURFDetector2)
  {
    //Load the input data
    /*TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/A/", "trijumpSD_shortcut.xml");
    vector<Frame*> SFrames = project.getFrames();
    for(int i = 0; i < SFrames.size(); i++) SFrames[i]->Resize(720);
    Frame* Pattern = SFrames[0];
    Skeleton SkeletonPattern = Pattern->getSkeleton();*/

    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/C/", "skier.xml");
    vector<Frame*> SFrames = project.getFrames();

    TestProjectLoader projectPattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Frames = projectPattern.getFrames();
    Frame* Pattern = Frames[1];
    Skeleton SkeletonPattern = Pattern->getSkeleton(); //Copying the manually marked skeleton
    
    // Create parameters
    map<string, float> params;
    params.emplace(std::pair<std::string, float>("markingLinearError", 10.0f));
    params.emplace(std::pair<std::string, float>("minHessian", 300.0f));
    //params.emplace(std::pair<std::string, float>("FixedWidthCells", 10.0f));
    //params.emplace(std::pair<std::string, float>("FixedLenghtCells", 10.0f));
    //params.emplace(std::pair<std::string, float>("useMask", 0.0f));
    //params.emplace("internalFrameHeight", 1000);

    // Run train
    SURFDetector2 D;
    long train_t0 = clock();
    D.train(SFrames, params);
    long train_t1 = clock();

    // Run detect
    SFrames[1]->setSkeleton(SkeletonPattern); //Copy skeleton from keyframe to frames[1]
    map<uint32_t, vector<LimbLabel>> Labels;
    long detect_t0 = clock();
    Labels = D.detect(SFrames[1], params, Labels);
    long detect_t1 = clock();

    /*cout << "Keypoints:" << endl;	
    for (int i = 0; i < D.Trained.PartKeypoints.size(); i++)
      cout << "Part[" << i << "]: " << D.Trained.PartKeypoints[i].size() << endl;*/

    cout << endl << " DETECTOR PARAMETERS\n\n";

    cout << "minHessian = " << D.parameters.minHessian << endl;
    cout << "searchDistCoeff = " << D.parameters.searchDistCoeff << endl;
    cout << "minTheta = " << D.parameters.minTheta << endl;
    cout << "maxTheta = " << D.parameters.maxTheta << endl;
    cout << "stepTheta = " << D.parameters.stepTheta << endl;
    cout << "uniqueLocationCandidates = " << D.parameters.uniqueLocationCandidates << endl;
    cout << "uniqueAngleCandidates = " << D.parameters.uniqueAngleCandidates << endl;

    cout << "isWeakThreshold = " << D.parameters.isWeakThreshold << endl;
    cout << "searchStepCoeff = " << D.parameters.searchStepCoeff << endl;

    cout << "markingLinearError = " << D.parameters.markingLinearError << endl;
    cout << "FixedWidthCells = " << D.parameters.FixedWidthCells << endl;
    cout << "FixedLenghtCells = " << D.parameters.FixedLenghtCells << endl;
    cout << "useDefaultCellsCount = " << D.parameters.useDefaultCellsCount << endl;

    // Set test parameters
    int TopLabelsCount = 4; //Size of "labels top list"
    float TolerableLinearError = 30; //Linear error in pixels
                                                                                     
    // Compare limb labels with ideal bodyparts from keyframe 
    map<int, vector<float>> LinearErrors = LabelsLinearErrors(Labels, SkeletonPattern);
    map<int, vector<float>> AngleErrors = LabelsAngleErrors(Labels, SkeletonPattern);

    // Write debug information into text file
    string OutputFileName = "DetectTest_SURF_B.txt";
    ofstream fout(OutputFileName);
    fout << "\n  DETECT RESULTS\n\n";
    PutSigificantErrors(fout, LinearErrors, AngleErrors, TopLabelsCount);
    PutLabels(fout, "", Labels, LinearErrors, AngleErrors, TopLabelsCount);
    PutLabels(fout, "", Labels, LinearErrors, AngleErrors, 0);
    fout.close();

    // Cheking the values and put test result
    vector<int> notFoundedParts = selectNotFoundedParts(SkeletonPattern, LinearErrors, Labels, TolerableLinearError, TopLabelsCount);
    cout << "\n EXECUTION TIME\n" << "\nTrain: " << clock_to_ms(train_t1 - train_t0) << " ms\n";
    cout << "Detect: " << clock_to_ms(detect_t1 - detect_t0) << " ms\n";

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
    cout << "\nLimbLabels saved in file: " << OutputFileName << endl << endl;

    SFrames.clear();
    //Frames.clear();
  }

  TEST(SURFDetectorExperiments, getPartPolygon)
  {
    Point2f p0(0.0f, 0.0f), p1(10.0f, 0.0f);
    float LWRatio = 1.3f;
    SURFDetector2 D;
    for (float angle = 0.0f; angle < 360.0f; angle++)
    {
      Point2f p0_ = spelHelper::rotatePoint2D(p0, Point2f(0.0f, 0.0f), angle);
      Point2f p1_ = spelHelper::rotatePoint2D(p1, Point2f(0.0f, 0.0f), angle);
      BodyPart bodyPart(0, "", 0, 1);
      bodyPart.setLWRatio(LWRatio);

      vector<Point2f> polygon = D.getPartPolygon(LWRatio, p0_, p1_);
      vector<Point2f> rect = bodyPart.getBodyPartRect(p0_, p1_).asVector();
      
      for (int i = 0; i < polygon.size(); i++)
      {
        EXPECT_NEAR(rect[i].x, polygon[i].x, 0.5f) << " angle = " << angle; 
        EXPECT_NEAR(rect[i].x, polygon[i].x, 0.5f) << " angle = " << angle;
      }
    }
  }

}