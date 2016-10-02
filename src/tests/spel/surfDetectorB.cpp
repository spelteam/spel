#include "surfDetectorB.hpp"
#include "surfDetector.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"


namespace SPEL
{
  void SURFDetector::SkeletonModel::clear(void)
  {
    PartCellsCount.clear();
    PartKeypointsCount.clear();
    Keypoints.clear();
    StudiedFramesID.clear();
    Descriptors.release();
  }

  SURFDetector::SkeletonModel::~SkeletonModel(void)
  {
    clear();
  }

  SURFDetector::SURFDetector(void)
  {
    m_id = 0x53440000;
  }

  SURFDetector::~SURFDetector(void)
  {
    Trained.clear();
  }

  void SURFDetector::setParameters(std::map<std::string, float> params)
  {
    if(Trained.StudiedFramesID.size() == 0)
    {
      params.emplace(COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN());
      parameters.minHessian = params.at(
         COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN().first);

      params.emplace(SPEL_SET_PARAMETER("markingLinearError", 10.0f));
      parameters.markingLinearError = params.at("markingLinearError");

      params.emplace(SPEL_SET_PARAMETER("FixedWidthCells", 0.0f));
      parameters.FixedWidthCells = static_cast<int>(params.at("FixedWidthCells"));

      params.emplace(SPEL_SET_PARAMETER("FixedLenghtCells", 0.0f));
      parameters.FixedLenghtCells = static_cast<int>(params.at("FixedLenghtCells"));

      params.emplace(SPEL_SET_PARAMETER("useDefaultCellsCount", 1.0f));
      parameters.useDefaultCellsCount = static_cast<bool>(params.at("useDefaultCellsCount"));
    }
    changeParameters(params);
  }

  void SURFDetector::changeParameters(std::map<std::string, float> params) const
  {
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

    parameters.searchDistCoeff = params.at(
       DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT().first);
    parameters.minTheta = params.at(DETECTOR_DETECT_PARAMETERS::MIN_THETA().first);
    parameters.maxTheta = params.at(DETECTOR_DETECT_PARAMETERS::MAX_THETA().first);
    parameters.stepTheta = params.at(DETECTOR_DETECT_PARAMETERS::STEP_THETA().first);
    parameters.uniqueLocationCandidates = params.at(
       DETECTOR_DETECT_PARAMETERS::UNIQUE_LOCATION_CANDIDATES_COEFFICIENT().first);
    parameters.uniqueAngleCandidates = params.at(
       DETECTOR_DETECT_PARAMETERS::UNIQUE_ANGLE_CANDIDATES_COEFFICIENT().first);
    parameters.isWeakThreshold = params.at(
       DETECTOR_DETECT_PARAMETERS::IS_WEAK_THRESHOLD().first);
    parameters.searchStepCoeff = params.at(
       DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT().first);
  }

  std::vector<cv::Point2f> SURFDetector::getPartPolygon(float LWRatio, cv::Point2f p0, cv::Point2f p1) const
  {
    std::vector<cv::Point2f> partRect;
    cv::Point2f d = p0 - p1;
    if (LWRatio <=0) DebugMessage(" LWRatio <= 0", 2);
    float k = 0.5f/LWRatio;
    float dx = k*(d.y);
    float dy = k*(-d.x);
    partRect.push_back(cv::Point2f(p0.x + dx, p0.y + dy));
    partRect.push_back(cv::Point2f(p1.x + dx, p1.y + dy));
    partRect.push_back(cv::Point2f(p1.x - dx, p1.y - dy));
    partRect.push_back(cv::Point2f(p0.x - dx, p0.y - dy));

    return partRect;
  }

  float SURFDetector::getLenght(std::vector<cv::Point2f> polygon) const
  {
    cv::Point2f d = polygon[1] - polygon[0];
    return sqrt(d.x*d.x + d.y*d.y);
  }

  float SURFDetector::getWidth(std::vector<cv::Point2f> polygon) const
  {
    cv::Point2f d = polygon[3] - polygon[0];
    return sqrt(d.x*d.x + d.y*d.y);
  }

  std::map<int, std::vector<cv::Point2f>> SURFDetector::getAllPolygons(Skeleton &skeleton) const
  {
    std::map<int, std::vector<cv::Point2f>> Rects;
    tree<BodyPart> PartTree = skeleton.getPartTree();
    for (tree<BodyPart>::iterator BP_iterator = PartTree.begin(); BP_iterator != PartTree.end(); BP_iterator++)
    {
      BodyJoint *j0 = skeleton.getBodyJoint(BP_iterator->getParentJoint());
      BodyJoint *j1 = skeleton.getBodyJoint(BP_iterator->getChildJoint());
      std::vector<cv::Point2f> Rect = getPartPolygon(BP_iterator->getLWRatio(), j0->getImageLocation(), j1->getImageLocation());
      Rects.emplace(std::pair<int, std::vector<cv::Point2f>>((*BP_iterator).getPartID(), Rect));
    }
    return Rects;
  }

  std::vector<int> SURFDetector::PolygonsPriority(std::map<int, std::vector<cv::Point2f>> partRects) const
  {
    typedef std::pair<int, float> PartArea;
    class ComparePartsArea
    {
    public:
      bool operator () (PartArea X, PartArea Y)
      {
        return (X.second < Y.second);
      }
    };

    std::vector<PartArea> PolygonsArea(partRects.size());
    for (int i = 0; i < partRects.size(); i++)
      PolygonsArea[i] = std::pair<int, float>(i, getLenght(partRects[i])*getWidth(partRects[i]));
    
    std::sort(PolygonsArea.begin(), PolygonsArea.end(), ComparePartsArea());
    std::vector<int> temp(partRects.size());
    for (int i = 0; i < partRects.size(); i++)
      temp[i] = PolygonsArea[i].first;

    PolygonsArea.clear();

    return temp;
  }

  std::vector<cv::KeyPoint> SURFDetector::SelectMaskKeypoints(cv::Mat &mask, std::vector<cv::KeyPoint> FrameKeypoints) const
  {
    std::vector<cv::KeyPoint> MaskKeypoins;
    for (unsigned int p = 0; p < FrameKeypoints.size(); p++)
      if (mask.at<uint8_t>(FrameKeypoints[p].pt) < 10)
        FrameKeypoints[p].class_id = -1;
      else
      {
        FrameKeypoints[p].class_id = -2;
        MaskKeypoins.push_back(FrameKeypoints[p]);
      }
    return MaskKeypoins;
  }

  int SURFDetector::PartCellIndex(int PartID, cv::Point2f pt, std::vector<cv::Point2f> polygon, cv::Size CellsCount) const
  {
    cv::Point2f dL = polygon[1] - polygon[0];
    cv::Point2f dW = polygon[3] - polygon[0];
    float l = sqrt(dL.x*dL.x + dL.y*dL.y) / static_cast<float>(CellsCount.height);
    float w = sqrt(dW.x*dW.x + dW.y*dW.y) / static_cast<float>(CellsCount.width);
    cv::Point2f d = dW;
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

  void SURFDetector::setCellsCount(std::map<int, std::vector<cv::Point2f>> &partPolygons, float markingError)
  {
    for (unsigned int k = 0; k < partPolygons.size(); k++)
    {
      cv::Size cellsCount(1, 1);
      if ( abs(markingError) > 1.0f)
      {
        float e = abs(ceil(markingError));
        int lenghtCells = trunc(getLenght(partPolygons[k]) / e);
        int widthCells = trunc(getWidth(partPolygons[k]) / e);
        if (widthCells < 1) widthCells = 1;
        if (lenghtCells < 1) lenghtCells = 1;
        cellsCount = cv::Size(widthCells, lenghtCells);
      }
      Trained.PartCellsCount.emplace(std::pair<int, cv::Size>(k, cellsCount));
    }
  }

  void SURFDetector::setFixedCellsCount(cv::Size partCellsCount)
  {
    for (unsigned int k = 0; k < Trained.PartCellsCount.size(); k++)
    {
      cv::Size temp = Trained.PartCellsCount[k];
      if (partCellsCount.width >= 0)
        temp = cv::Size(partCellsCount.width, temp.height);
      if (partCellsCount.height >= 0)
        temp = cv::Size(temp.width, partCellsCount.height);
      Trained.PartCellsCount[k] = temp;
    }
  }

  void SURFDetector::setDefaultCellsCount(void)
  {
    std::vector<cv::Size> Default = {cv::Size(3, 3), cv::Size(1, 3), cv::Size(1, 3), cv::Size(1, 2), cv::Size(1, 5),
                                     cv::Size(1, 5), cv::Size(2, 5), cv::Size(2, 5), cv::Size(5, 5), cv::Size(2, 4),
                                     cv::Size(2, 4), cv::Size(2, 4), cv::Size(2, 4), cv::Size(2, 4), cv::Size(2, 4),
                                     cv::Size(1, 3), cv::Size(1, 3), cv::Size(2, 3), cv::Size(2, 3) };
    for (unsigned int k = 0; k < Default.size(); k++)
      Trained.PartCellsCount.emplace(std::pair<int, cv::Size>(k, Default[k]));
  }

  void SURFDetector::SingleFrameTrain(Frame* frame)
  {
    // Create frame keypoints
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SurfFeatureDetector::create(parameters.minHessian);
    std::vector<cv::KeyPoint> FrameKeypoints;
    cv::Mat image = frame->getImage();
    cv::Mat Mask = frame->getMask();
    detector->detect(image, FrameKeypoints);

    // Select mask keypoints
    std::vector<cv::KeyPoint> MaskKeypoins = SelectMaskKeypoints(Mask, FrameKeypoints);

    // Calculate part rects
    Skeleton skeleton = frame->getSkeleton();
    std::map<int, std::vector<cv::Point2f>> partRects = getAllPolygons(skeleton);

    // Sorting polygons by area
    std::vector<int> SortedIndexes = PolygonsPriority(partRects);

    // Create part models if current frame is a first keyframe in the sequence
    if (Trained.PartCellsCount.size() == 0)
    {
      float e = parameters.markingLinearError;
      if(parameters.useDefaultCellsCount)
        setDefaultCellsCount();
      else
        setCellsCount(partRects, e);
      if (parameters.FixedWidthCells >= 1 || parameters.FixedLenghtCells >= 1)
        setFixedCellsCount(cv::Size(parameters.FixedWidthCells, parameters.FixedLenghtCells));
    }
    if (Trained.PartKeypointsCount.size() == 0)
      for (unsigned int k = 0; k < partRects.size(); k++)
        Trained.PartKeypointsCount.emplace(std::pair<int, int>(k, 0));

    // Create skeleton model
    for (unsigned int p = 0; p < MaskKeypoins.size(); p++)
      if(MaskKeypoins[p].class_id !=-1)
        for (unsigned int k = 0; k < SortedIndexes.size(); k++)
        {
          int PartID = SortedIndexes[k];
          std::vector<cv::Point2f> partPolygon = partRects[PartID];
          cv::Size partCellsCount = Trained.PartCellsCount[PartID];
          if (pointPolygonTest(partPolygon, MaskKeypoins[p].pt, false) > 0)
          {
            cv::Point2f pt = MaskKeypoins[p].pt;
            MaskKeypoins[p].class_id = PartCellIndex(PartID, pt,  partPolygon, partCellsCount);
            Trained.PartKeypointsCount.at(SortedIndexes[k])++;
            Trained.Keypoints.push_back(MaskKeypoins[p]);
          }
        }
    cv::Mat FrameDescriptors;
    cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    extractor->compute(image, Trained.Keypoints, Trained.Descriptors);
    Trained.StudiedFramesID.push_back(frame->getID());

    // Clear
    detector->clear();
    extractor->clear();
    SortedIndexes.clear();
  }
 
  void SURFDetector::Train(std::vector<Frame*> frames)
  {
    Trained.clear();

    DebugMessage(" SURFDetector Train started", 2);

    // Training on all keyframes from current sequence
    for (int i = 0; i < frames.size(); i++)
      if (frames[i]->getFrametype() == KEYFRAME)
        SingleFrameTrain(frames[i]);

    DebugMessage(" SURFDetector Train completed", 2);
  }
  void SURFDetector::train(const std::vector<Frame*> &frames, std::map<std::string, float> params)
  {
    setParameters(params);
    Train(frames);
  }

  std::vector<int> SURFDetector::getStudiedFramesID(void) const
  {
    return Trained.StudiedFramesID;
  }

  class CompareLabels
  {
  public:
    bool operator () (LimbLabel X, LimbLabel Y)
    {
      return (X.getAvgScore() < Y.getAvgScore());
    }
  };

  std::map<uint32_t, std::vector<LimbLabel>> SURFDetector::Detect(Frame* frame) const
  {
    bool useMulct = false;
    bool CheckMatches = false;

    DebugMessage(" SURFDetector Detect started", 2);
    cv::Mat image = frame->getImage();
    cv::Mat Mask = frame->getMask();

    // Calculate frame keypoints
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SurfFeatureDetector::create(parameters.minHessian);
    std::vector<cv::KeyPoint> FrameKeypoints;
    detector->detect(image, FrameKeypoints);

    // Select mask keypoints
    std::vector<cv::KeyPoint> MaskKeypoins = SelectMaskKeypoints(Mask, FrameKeypoints);

    std::stringstream s1;
    s1 << " Mask keypoints count: " << MaskKeypoins.size() << std::endl;
    DebugMessage(s1.str(), 2);
    s1.clear();

    // Extract descriptors
    cv::Mat FrameDescriptors;
    cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    extractor->compute(image, MaskKeypoins, FrameDescriptors);

    // Create skeleton model for the current frame
    SkeletonModel Local;
    Local.Keypoints = MaskKeypoins;
    Local.Descriptors = FrameDescriptors;
    Skeleton skeleton = frame->getSkeleton();
    std::map<int, std::vector<cv::Point2f>> PartRects = getAllPolygons(skeleton);
    for (int i = 0; i < PartRects.size(); i++)
      Local.PartKeypointsCount.emplace(i, 0);

    // Create matches
    int n = 2; //matches per keypoint
    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(FrameDescriptors, Trained.Descriptors, matches, n);
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

    std::map<uint32_t, std::vector<LimbLabel>> Labels;

    for (int id = 0; id < PartRects.size(); id++)
    {
      if(Trained.PartKeypointsCount.at(id) == 0)
      {
        std::stringstream s;
        s << " Body Part " << id << " model is empty" << std::endl;
        DebugMessage(s.str(), 2);
        s.clear();
      }
      if(Trained.PartKeypointsCount.at(id) > 0) // or " > threshold"
      {
        std::vector<cv::Point2f> partPolygon = PartRects[id]; 	  
        float partLenght = getLenght(partPolygon);
        float partWidth = getWidth(partPolygon);
        std::vector<cv::Point2f> LabelPolygon(4), RotatedPolygon(4);

        cv::Point2f PartCenter = 0.5f*(partPolygon[3] + partPolygon[1]);
        float SearchDist = parameters.searchDistCoeff*partLenght;
        float minStep = abs(parameters.searchStepCoeff*partWidth);
        minStep = std::max(minStep, 2.0f);

        cv::Size CellsCount = Trained.PartCellsCount[id];

        int LabelsPerPart = 0;
        std::vector<LimbLabel> PartLabels;

        float PartAngle = static_cast<float>(spelHelper::getAngle(partPolygon[0], partPolygon[1]));
        for (float angleShift = - parameters.minTheta; angleShift < parameters.maxTheta; angleShift += parameters.stepTheta)
        { 
          // Rotation of the part polygon
          float LabelAngle = PartAngle + angleShift;
          for(int t = 0; t < LabelPolygon.size(); t++)
            RotatedPolygon[t] = spelHelper::rotatePoint2D(partPolygon[t], PartCenter, angleShift);
          //b++;
          for(float x = -SearchDist; x < SearchDist; x += minStep)
            for(float y = - SearchDist; y < SearchDist; y += minStep)
            {
              //g++;
              LabelsPerPart++;

              // Shift label polygon
              for (int t = 0; t < LabelPolygon.size(); t++)
                LabelPolygon[t] = RotatedPolygon[t] + cv::Point2f(x, y);
              cv::Point2f LabelCenter = PartCenter + cv::Point2f(x, y);

              // Calculate score for current label
              double LabelScore = 0;

              for(unsigned int p = 0; p < MaskKeypoins.size(); p++)
                if(pointPolygonTest(LabelPolygon, MaskKeypoins[p].pt, false) > 0)
                {
                  int partCellID = PartCellIndex(id, MaskKeypoins[p].pt, LabelPolygon, CellsCount);
                  if(Trained.Keypoints[matches[p][0].trainIdx].class_id == partCellID)
                  {
                    LabelScore = LabelScore + matches[p][0].distance;
                    Local.PartKeypointsCount[id]++;
                  }
                }

              // Normalize and inverse score
              if(Local.PartKeypointsCount[id] > 0)
                LabelScore = 1.0 - LabelScore / static_cast<double>(Trained.PartKeypointsCount[id]);
              else
                LabelScore = INFINITY;

              // Create LimbLabel
              Score score(static_cast<float>(LabelScore), "");
              std::vector<Score> scores;
              scores.push_back(score);
              LimbLabel Label(id, LabelCenter, LabelAngle, LabelPolygon, scores);
              PartLabels.push_back(Label);
              scores.clear();
            }
        }
        sort(PartLabels.begin(), PartLabels.end(), CompareLabels());
        Labels.emplace(std::pair<int, std::vector<LimbLabel>>(id, PartLabels));
        PartLabels.clear();
      
        std::stringstream s;
        s << " Limb Labels count per Body Part " << id <<": " << LabelsPerPart << std::endl;
        DebugMessage(s.str(), 2);
        s.clear();
      }
    }
    detector->clear();

    DebugMessage(" SURFDetector Detect completed", 2);
    return Labels;
  }

  std::map<uint32_t, std::vector<LimbLabel>> SURFDetector::detect(Frame* frame,
       std::map<std::string, float> params, 
      const std::map<uint32_t, std::vector<LimbLabel>> &limbLabels) const
  {
    changeParameters(params);
    std::map<uint32_t, std::vector<LimbLabel>> NewLabels;
    NewLabels = SURFDetector::Detect(frame);
    // Need labels merging
    //FilterLimbLabels(...)
    return NewLabels;
  }

  // It is gag
  LimbLabel SURFDetector::generateLabel(const BodyPart &bodyPart,
      Frame *workFrame, const cv::Point2f &parent, const cv::Point2f &child,
     DetectorHelper *detectorHelper, std::map<std::string, float> params) const
  {
    // Compute frame keypoints, descriptors and matches
    cv::Mat image = workFrame->getImage();
    cv::Mat Mask = workFrame->getMask();
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SurfFeatureDetector::create(parameters.minHessian);
    std::vector<cv::KeyPoint> FrameKeypoints;
    detector->detect(image, FrameKeypoints);
    std::vector<cv::KeyPoint> MaskKeypoins = SelectMaskKeypoints(Mask, FrameKeypoints);

    cv::Mat FrameDescriptors;
    cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    extractor->compute(image, MaskKeypoins, FrameDescriptors);

    int n = 2;
    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(FrameDescriptors, Trained.Descriptors, matches, n);

    float maxDist = 0.0f;
    for(int i = 0; i < matches.size(); i++)
      for(int k = 0; k < n; k++)
        if (matches[i][k].distance > maxDist)
          maxDist = matches[i][k].distance;

    for(int i = 0; i < matches.size(); i++)
      for(int k = 0; k < n; k++)
        matches[i][k].distance = 1.0f - matches[i][k].distance/maxDist;

    // Calculate score for current label
    int id = bodyPart.getPartID();
    float angle = spelHelper::getAngle(parent, child);
    cv::Point2f center = 0.5f*(parent + child);
    std::vector<cv::Point2f> polygon = getPartPolygon(bodyPart.getLWRatio(), parent, child);
    float LabelScore = 0.0f;

    for (unsigned int p = 0; p < MaskKeypoins.size(); p++)
      if (pointPolygonTest(polygon, MaskKeypoins[p].pt, false) > 0)
      {
        cv::Size CellsCount = Trained.PartCellsCount[id];
        int partCellID = PartCellIndex(id, MaskKeypoins[p].pt, polygon, CellsCount);
        if (Trained.Keypoints[matches[p][0].trainIdx].class_id == partCellID)
          LabelScore = LabelScore + matches[p][0].distance;
      }

    // Create LimbLabel
    Score score(LabelScore, "");
    std::vector<Score> scores;
    scores.push_back(score);
    LimbLabel Label(id, center, angle, polygon, scores);

    return Label;
  }
}