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
  };

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

  void SingleFrameTrain_(Frame* frame, SkeletonModel_ &X, Parameters_ P, Size CellsCount)
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
            MaskKeypoins[p].class_id = PartCellIndex(pt, PartID, PartPolygon, CellsCount);
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

  void Train_(vector<Frame*> frames, SkeletonModel_ &X, Parameters_ P, Size CellsCount = Size(10, 10))
  {
    DebugMessage(" SURFDetector experiments Train_2 started", 2);

    // Training on all keyframes
    for(int i = 0; i < frames.size(); i++)
      if (frames[i]->getFrametype() == KEYFRAME)
        SingleFrameTrain_(frames[i], X, P, CellsCount);
 
    DebugMessage(" SURFDetector experiments Train_2 completed", 2);
  }

  map<uint32_t, vector<LimbLabel>> Detect_(Frame* frame, SkeletonModel_ &Trained, Parameters_ P, Size CellsCount = Size(10, 10))
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

        float PartAngle = static_cast<float>(90.0 - 180.0*atan2(dL.y, dL.x)/M_PI);
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
                    int partCellID = PartCellIndex(MaskKeypoins[p].pt, id, LabelPolygon, CellsCount);
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
    Size CellsCount(4, 6);
    //Load the input data
    //TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/A/", "trijumpSD_shortcut.xml");
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/C/", "skier.xml");
    vector<Frame*> SFrames = project.getFrames();
    /*for (int i = 0; i < SFrames.size(); i++)
      SFrames[i]->Resize(1024);
    Frame* Pattern = SFrames[1];*/

    TestProjectLoader project_pattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Frames = project_pattern.getFrames();
    Frame* Pattern = Frames[1];

    //Copy image and skeleton from first keyframe
    int FirstKeyframe = 0;
    Mat image = SFrames[FirstKeyframe]->getImage();
    Mat mask = SFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = SFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    // Create parameters
    map<string, float> params;
    Parameters_ P = SetDetectParameters_(params);

    // Run experimental train
    SkeletonModel_ X;
    Train_(SFrames, X, P, CellsCount);

    // Run experimental detect
    SFrames[1]->setSkeleton(Pattern->getSkeleton()); // Copy skeleton from keyframe to frames[1]
    map<uint32_t, vector<LimbLabel>> limbLabels = Detect_(SFrames[1], X, P, CellsCount);

    // Create output file
    string OutputFileName = "SurDetectorfExperiments_B.txt";
    ofstream fout(OutputFileName);

    // Output top of "limbLabels" into text file
    fout << "\nTop Labels, sorted by part id:\n\n";
    for (unsigned int i = 0; i < limbLabels.size(); i++) // For all body parts
    {
      for (unsigned int k = 0; (k < limbLabels[i].size()) && (k < 4); k++) // For all scores of this bodypart
      {
        Point2f p0, p1;
        limbLabels[i][k].getEndpoints(p0, p1); // Copy the Limblabel points
        fout << "  " << i << ":" << " limbID = " << limbLabels[i][k].getLimbID() << ", Angle = " << limbLabels[i][k].getAngle() << ", Points = {" << p0 << ", " << p1 << "}, AvgScore = " << limbLabels[i][k].getAvgScore() << ", Scores = {";
        vector<Score> scores = limbLabels[i][k].getScores(); // Copy the Label scores
        for (unsigned int t = 0; t < scores.size(); t++)
          fout << scores[t].getScore() << ", "; // Put all scores of the Label
        fout << "}\n";
      }
      fout << endl;
    }

    // Copy coordinates of BodyParts
    Skeleton SkeletonPattern = Pattern->getSkeleton();
    map<int, pair<Point2f, Point2f>> PartLocation = getPartLocations(SkeletonPattern);

    // Compare labels with ideal bodyparts from keyframe, and output debug information 
    float TolerableCoordinateError = 30; // Linear error in pixels
    cout << "TolerableCoordinateError = " << TolerableCoordinateError << endl;
    int TopListLabelsCount = 4; // Size of "labels top list"
    map<int, vector<LimbLabel>> effectiveLabels;
    vector<int> WithoutGoodLabelInTop;
    bool EffectiveLabbelsInTop = true;

    fout << "-------------------------------------\nAll labels, with distance from the ideal body part: \n";

    for (unsigned int id = 0; id < limbLabels.size(); id++)
    {
      fout << "\nPartID = " << id << ":\n";
      Point2f l0, l1, p0, p1, delta0, delta1;
      vector<LimbLabel> temp;
      p0 = PartLocation[id].first; // Ideal boby part point
      p1 = PartLocation[id].second; // Ideal boby part point
      for (int k = 0; k < static_cast<int>(limbLabels[id].size()); k++)
      {
        limbLabels[id][k].getEndpoints(l0, l1); // Label points
        delta0 = l0 - p0;
        delta1 = l1 - p1;
        float error_A = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
        delta0 = l0 - p1;
        delta1 = l1 - p0;
        float error_B = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
        float error = min(error_A, error_B); // Distance between ideal body part and label
        if (error <= TolerableCoordinateError && limbLabels[id][k].getAvgScore() >= 0) // Label is "effective" if it has small error and not less than zero  Score  value
          temp.push_back(limbLabels[id][k]); // Copy effective labels
        // Put linear errors for all Lalbels into text file, copy indexes of a "badly processed parts"
        fout << "    PartID = " << id << ", LabelIndex = " << k << ":    AvgScore = " << limbLabels[id][k].getAvgScore() << ", LinearError = " << error << endl;
        if (k == TopListLabelsCount - 1)
        {
          fout << "    //End of part[" << id << "] top labels list\n";
          if (!(skeleton.getBodyPart(id)->getIsOccluded()))
            if (temp.size() < 1)
            {
              EffectiveLabbelsInTop = false; // false == Present not Occluded bodyparts, but no nave "effective labels" in the top of list
              WithoutGoodLabelInTop.push_back(id); // Copy index of not Occluded parts, wich no have "effective labels" in the top of labels list
            }
        }
      }
      effectiveLabels.emplace(pair<int, vector<LimbLabel>>(id, temp));
    }

    //Output top of "effectiveLabels" into text file
    fout << "\n-------------------------------------\n\nTrue Labels:\n\n";
    for (unsigned int i = 0; i < effectiveLabels.size(); i++)
    {
      for (unsigned int k = 0; k < effectiveLabels[i].size(); k++)
      {
        Point2f p0, p1;
        limbLabels[i][k].getEndpoints(p0, p1);
        fout << "  limbID = " << effectiveLabels[i][k].getLimbID() << ", Angle = " << effectiveLabels[i][k].getAngle() << ", Points = {" << p0 << ", " << p1 << "}, AvgScore = " << effectiveLabels[i][k].getAvgScore() << ", Scores = {";
        vector<Score> scores = effectiveLabels[i][k].getScores();
        for (unsigned int t = 0; t < scores.size(); t++)
          fout << scores[t].getScore() << ", ";
        fout << "}\n";
      }
      fout << endl;
    }

    fout.close();
    cout << "\nLimbLabels saved in file:" << OutputFileName << endl;

    // Output messages 
    if (!EffectiveLabbelsInTop) cout << endl << " SurfDetector_Tests.detect:" << endl;
    EXPECT_TRUE(EffectiveLabbelsInTop);
    if (!EffectiveLabbelsInTop)
    {
      cout << "Body parts with id: ";
        for (unsigned int i = 0; i < WithoutGoodLabelInTop.size(); i++)
        {
          cout << WithoutGoodLabelInTop[i];
          if (i != WithoutGoodLabelInTop.size() - 1) cout << ", ";
        }
      cout << " - does not have effective labels in the top of labels list." << endl;
    }
    if (!EffectiveLabbelsInTop) cout << endl;

    SFrames.clear();
    Frames.clear();
  }

}