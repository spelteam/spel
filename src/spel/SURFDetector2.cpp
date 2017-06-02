// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
#include "SURFDetector2.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"
#include "spelGeometry.hpp"

namespace SPEL
{
  void SURFDetector2::SkeletonModel::clear(void)
  {
    PartCellsCount.clear();
    PartKeypoints.clear();
    Keypoints.clear();
    StudiedFramesID.clear();
    Descriptors.release();
  }

  SURFDetector2::SkeletonModel::~SkeletonModel(void)
  {
    clear();
  }

  SURFDetector2::SURFDetector2(void)
  {
    m_id = 0x53440000;
  }

  SURFDetector2::SURFDetector2(std::map<std::string, float> params)
  {
    m_id = 0x53440000;
    setParameters(params);
  }

  SURFDetector2::~SURFDetector2(void)
  {
    Trained.clear();
  }

  // The set of parameters which can be changed only before training
  void SURFDetector2::setParameters(std::map<std::string, float> params)
  {
    if(Trained.StudiedFramesID.size() == 0)
    {
      params.emplace(COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN());
      parameters.minHessian = params.at(
         COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN().first);

      params.emplace(SPEL_SET_PARAMETER("markingLinearError", 10.0f));
      parameters.markingLinearError = params.at("markingLinearError");

      params.emplace(SPEL_SET_PARAMETER("minCellSize", 2.0f));
      parameters.minCellSize = params.at("minCellSize");

      params.emplace(SPEL_SET_PARAMETER("FixedWidthCells", 3.0f));
      parameters.FixedWidthCells = static_cast<int>(params.at("FixedWidthCells"));

      params.emplace(SPEL_SET_PARAMETER("FixedLenghtCells", 5.0f));
      parameters.FixedLenghtCells = static_cast<int>(params.at("FixedLenghtCells"));

      params.emplace(SPEL_SET_PARAMETER("useDefaultCellsCount", 1.0f));
      parameters.useDefaultCellsCount = (params.at("useDefaultCellsCount") >= 0.5f); // float to bool

      params.emplace(SPEL_SET_PARAMETER("useMask", 1.0f));
      parameters.useMask = (params.at("useMask") >= 0.5f); // float to bool

      params.emplace("maxFrameHeight", 0.0f);
      params.emplace("externalFrameHeight", params.at("maxFrameHeight"));
      parameters.externalFrameHeight = static_cast<int>(params.at("externalFrameHeight"));

      params.emplace("internalFrameHeight", 0.0f);
      parameters.internalFrameHeight = static_cast<int>(params.at("internalFrameHeight"));

      /*params.emplace("adjustSolves", false);
      parameters.adjustSolves = static_cast<bool>(params.at("adjustSolves"));
      */
    }
    changeParameters(params);
  }

  // The set of parameters which can be changed after training
  void SURFDetector2::changeParameters(std::map<std::string, float> params) const
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

  Frame* SURFDetector2::preparedFrame(Frame* frame) const
  {
    cv::Mat mask = frame->getMask();
    cv::Mat image = frame->getImage();
    Frame * temp = 0;
    int frametype = frame->getFrametype();
    switch (frametype)
    {
      case UNDEFINED: temp = new Interpolation(); break;
      case KEYFRAME: temp = new Keyframe(); break;
      case LOCKFRAME: temp = new Lockframe(); break;
      case INTERPOLATIONFRAME: temp = new Interpolation(); break;
      default: new Interpolation(); break;
    }
    temp->setID(frame->getID());
    temp->setGroundPoint(frame->getGroundPoint());
    temp->setParentFrameID(frame->getParentFrameID());
    temp->setSkeleton(frame->getSkeleton());

    int height = image.size().height;
    int width = image.size().width;
    if (height != 0 && parameters.internalFrameHeight != 0 && parameters.internalFrameHeight != height)
    {
      float scale = static_cast<float>(parameters.internalFrameHeight) /
        static_cast<float>(height);
      temp->Scale(scale);
      cv::Mat scaledImage;
      cv::Mat scaledMask;
      cv::Size frameSize = cv::Size(static_cast<int>(width*scale), static_cast<int>(height*scale));
      cv::resize(image, scaledImage, frameSize);
      cv::resize(mask, scaledMask, frameSize);
      bool cacheFile = false;
      temp->setImage(scaledImage, cacheFile);
      temp->setMask(scaledMask, cacheFile);
      scaledImage.release();
      scaledMask.release();
    }
    else
    {
      temp->setImage(image.clone());
      temp->setMask(mask.clone());
    }

    if (frame->GetImagePath().empty()) frame->cacheImage(); // !??????
    frame->UnloadImage();
    if (frame->GetImagePath().empty()) frame->cacheMask(); // !??????
    frame->UnloadMask();

    return temp;
  }

  void SURFDetector2::setAutoInternalFrameHeight(std::vector<Frame*> frames)
  {
    int maxHeight = 0;
    for (uint32_t i = 0; i < frames.size(); i++)
    {
      int height = frames[i]->getImageSize().height;
      if(height > maxHeight) // !??????
        maxHeight = height;
      if (frames[i]->GetImagePath().empty()) frames[i]->cacheImage(); // !??????
      frames[i]->UnloadImage();      
    }
    parameters.internalFrameHeight = maxHeight;
  }
  // Moved to "spelGeometry.cpp":
  // std::vector<cv::Point2f> SURFDetector2::getPartPolygon(float LWRatio, cv::Point2f p0, cv::Point2f p1) const;
  // float SURFDetector2::getLenght(std::vector<cv::Point2f> polygon) const;
  // float SURFDetector2::getWidth(std::vector<cv::Point2f> polygon) const;
  // std::map<int, std::vector<cv::Point2f>> SURFDetector2::getAllPolygons(Skeleton &skeleton) const;

  std::vector<int> SURFDetector2::PolygonsPriority(std::map<int, std::vector<cv::Point2f>> partRects) const
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
      PolygonsArea[i] = std::pair<int, float>(i, getPartLenght(partRects[i])*getPartWidth(partRects[i]));
    
    std::sort(PolygonsArea.begin(), PolygonsArea.end(), ComparePartsArea());
    std::vector<int> temp(partRects.size());
    for (int i = 0; i < partRects.size(); i++)
      temp[i] = PolygonsArea[i].first;

    PolygonsArea.clear();

    return temp;
  }

  std::vector<cv::KeyPoint> SURFDetector2::SelectMaskKeypoints(cv::Mat &mask, std::vector<cv::KeyPoint> FrameKeypoints) const
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

  int SURFDetector2::PartCellIndex(int PartID, cv::Point2f pt, std::vector<cv::Point2f> polygon, cv::Size CellsCount) const
  {
    if(CellsCount == cv::Size(0,0))
      CellsCount = Trained.PartCellsCount[PartID];
    double L = getPartLenght(polygon);
    double W = getPartWidth(polygon);
    double l = L / static_cast<double>(CellsCount.height);
    double w = W / static_cast<double>(CellsCount.width);
    double a = W;
    cv::Point2f d = pt - polygon[0];
    double b = sqrt(d.x*d.x + d.y*d.y);
    d = pt - polygon[3];
    double c = sqrt(d.x*d.x + d.y*d.y);
    double p = 0.5*(a + b + c);
    double dl = 2 * sqrt(p*(p - a)*(p - b)*(p - c)) / a;
    double dw = sqrt(b*b - dl*dl);
    int id = -2;
    if ((dl <= L) && (dw <= W))
    {
      int nw = static_cast<int>(trunc(dw / w));
      int nl = static_cast<int>(trunc(dl / l));
      if (dl == L) dl--;
      if (dw == W) dw--;
      id = PartID * partCellsLimit + /*CellsCount.width*CellsCount.height + */CellsCount.width*nl + nw;
    }

    return id;
  }

  int SURFDetector2::PartIndex(int CellIndex) const
  {
    return static_cast<int>(trunc(float(CellIndex) / float(partCellsLimit)));
  }

  void SURFDetector2::setCellsCount(std::map<int, std::vector<cv::Point2f>> &partPolygons, float markingError)
  {
    for (unsigned int k = 0; k < partPolygons.size(); k++)
    {
      cv::Size cellsCount(1, 1);
      if ( abs(markingError) > 1.0f)
      {
        float e = abs(ceil(markingError));
        int lenghtCells = static_cast<int>(trunc(getPartLenght(partPolygons[k]) / e));
        int widthCells = static_cast<int>(trunc(getPartWidth(partPolygons[k]) / e));
        if (widthCells < 1) widthCells = 1;
        if (lenghtCells < 1) lenghtCells = 1;
        cellsCount = cv::Size(widthCells, lenghtCells);
      }
      Trained.PartCellsCount.emplace(std::pair<int, cv::Size>(k, cellsCount));
    }
  }

  void SURFDetector2::setFixedCellsCount(cv::Size partCellsCount)
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

  void SURFDetector2::setDefaultCellsCount(void)
  {
    std::vector<cv::Size> Default = {cv::Size(3, 3), cv::Size(1, 3), cv::Size(1, 3), cv::Size(1, 2), cv::Size(1, 5),
                                     cv::Size(1, 5), cv::Size(2, 5), cv::Size(2, 5), cv::Size(5, 5), cv::Size(2, 4),
                                     cv::Size(2, 4), cv::Size(2, 4), cv::Size(2, 4), cv::Size(2, 4), cv::Size(2, 4),
                                     cv::Size(1, 3), cv::Size(1, 3), cv::Size(2, 3), cv::Size(2, 3) };
    for (unsigned int k = 0; k < Default.size(); k++)
      Trained.PartCellsCount.emplace(std::pair<int, cv::Size>(k, Default[k]));
  }

  void SURFDetector2::correctingCellsSize(std::map<int, std::vector<cv::Point2f>> &partPolygons)
  {
    for (unsigned int i = 0; i < partPolygons.size(); i++)
    {
      float lenght = getPartLenght(partPolygons[i]);
      float widht = getPartWidth(partPolygons[i]);
      float nl = static_cast<float>(Trained.PartCellsCount[i].height);
      float nw = static_cast<float>(Trained.PartCellsCount[i].width);
      if (lenght/nl < parameters.minCellSize)
        nl = trunc(lenght / parameters.minCellSize);
      if (widht /nw < parameters.minCellSize)
        nl = trunc(widht / parameters.minCellSize);
      Trained.PartCellsCount[i] = cv::Size(static_cast<int>(nw), static_cast<int>(nl));
    }
  }

  //========================================
  // Functions from "imagesimilaritymatrix.cpp"
  // Moved to "spelGeometry.cpp"

  //======================================

  std::vector<cv::KeyPoint> SURFDetector2::detectKeypoints(Frame* frame, bool useMask) const
  {
    cv::Mat Image = frame->getImage();

    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SurfFeatureDetector::create(parameters.minHessian);
    std::vector<cv::KeyPoint> Keypoints;
    if (useMask == true)
    {
      // Detect ROI keypoints
      cv::Mat Mask = frame->getMask();
      cv::Rect ROI = toROIRect(SearchROI(Mask));
      correctROI(ROI, Mask.size());
      //Image.adjustROI(2, 2, 2, 2);
      std::vector<cv::KeyPoint> FrameKeypoints;
      detector->detect(Image(ROI), FrameKeypoints);
      //cv::imwrite("zROI.jpg", Image(ROI));
      for(unsigned int i = 0; i < FrameKeypoints.size(); i++)
      FrameKeypoints[i].pt += cv::Point2f(static_cast<float>(ROI.x), static_cast<float>(ROI.y));

      // Select mask keypoints
      Keypoints = SelectMaskKeypoints(Mask, FrameKeypoints);
    }
    else
    {
      detector->detect(Image, Keypoints);
      for (unsigned int p = 0; p < Keypoints.size(); p++)
        Keypoints[p].class_id = -2;
    }
    detector->clear();

    return Keypoints;
  }

  void SURFDetector2::SingleFrameTrain(Frame* frame_)
  {
    /*if (parameters.externalFrameHeight == 0)
      parameters.externalFrameHeight = frame_->getImageSize().height; // !??????*/

    Frame* frame = frame_;
    bool scalingFrame = (parameters.internalFrameHeight != 0);
    scalingFrame = (scalingFrame && parameters.internalFrameHeight != frame_->getImageSize().height);
    if(scalingFrame)
      frame = preparedFrame(frame_);

    // Create frame keypoints
    //long t0 = clock();
    cv::Mat image = frame->getImage();
    std::vector<cv::KeyPoint> Keypoints = detectKeypoints(frame, parameters.useMask);
    //long t1 = clock();
    //std::cout << "xfeatures::detect time = " << t1 - t0 << std::endl;

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
      correctingCellsSize(partRects);
    }
    if (Trained.PartKeypoints.size() == 0)
      for (unsigned int k = 0; k < partRects.size(); k++)
        Trained.PartKeypoints.emplace(std::pair<int, std::vector<int>>(k, std::vector<int>()));

    // Create skeleton model
    for (unsigned int p = 0; p < Keypoints.size(); p++)
      if(Keypoints[p].class_id <-1)
        for (unsigned int k = 0; k < SortedIndexes.size(); k++)
        {
          int PartID = SortedIndexes[k];
          std::vector<cv::Point2f> partPolygon = partRects[PartID];
          cv::Size partCellsCount = Trained.PartCellsCount[PartID];
          if (pointPolygonTest(partPolygon, Keypoints[p].pt, false) > 0)
          {
            cv::Point2f pt = Keypoints[p].pt;
            Keypoints[p].class_id = PartCellIndex(PartID, pt,  partPolygon, partCellsCount);
            Trained.Keypoints.push_back(Keypoints[p]);
            Trained.PartKeypoints[PartID].push_back(p);
          }
        }
    cv::Mat FrameDescriptors;
    cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    extractor->compute(image, Trained.Keypoints, Trained.Descriptors);
    Trained.StudiedFramesID.push_back(frame->getID());

    // Clear
    extractor->clear();
    SortedIndexes.clear();

    if (scalingFrame)
      delete frame;
    else
    {
      if (frame->GetImagePath().empty()) frame->cacheImage(); // !??????
      frame->UnloadImage();
      if (frame->GetImagePath().empty()) frame->cacheMask(); // !??????
      frame->UnloadMask();	  
    }
  }

  void SURFDetector2::Train(std::vector<Frame*> frames)
  {
    Trained.clear();

    DebugMessage(" SURFDetector Train started", 2);

    // Training on all keyframes from current sequence
    for (int i = 0; i < frames.size(); i++)
      if (frames[i]->getFrametype() == KEYFRAME)
        SingleFrameTrain(frames[i]);

    DebugMessage(" SURFDetector Train completed", 2);
  }
  void SURFDetector2::train(const std::vector<Frame*> &frames, std::map<std::string, float> params)
  {
    setParameters(params);
    /*if(parameters.externalFrameHeight == 0)
    {
      parameters.externalFrameHeight = frames[0]->getImageSize().height; // !??????
      params.at("externalFrameHeight") = parameters.externalFrameHeight; // !??????
    }*/
    Train(frames);
  }

  std::vector<int> SURFDetector2::getStudiedFramesID(void) const
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

  std::map<uint32_t, std::vector<LimbLabel>> SURFDetector2::generatePartsLabels(Skeleton approximation) const
  {
    std::map<uint32_t, std::vector<LimbLabel>> Labels;

    std::map<int, std::vector<cv::Point2f>> PartRects = getAllPolygons(approximation);
    for (int id = 0; id < PartRects.size(); id++)
    {
      std::vector<LimbLabel> PartLabels;

      // Get part polygon from current interpolationframe
      std::string partID = std::to_string(id); // for messages
      std::vector<cv::Point2f> partPolygon = PartRects[id];
      cv::Point2f PartCenter = getPartCenter(partPolygon);
      float PartAngle = static_cast<float>(spelHelper::getAngle(partPolygon[0], partPolygon[1]));
          
      float partLenght = getPartLenght(partPolygon);
      float partWidth = getPartWidth(partPolygon);
      std::vector<cv::Point2f> LabelPolygon(4), RotatedPolygon(4);

      float SearchDist = parameters.searchDistCoeff*partLenght;
      float minStep = abs(parameters.searchStepCoeff*partWidth);
      minStep = std::max(minStep, 2.0f);

      // Create limbLabels
      for (float angleShift = -parameters.minTheta; angleShift < parameters.maxTheta; angleShift += parameters.stepTheta)
      {
        // Rotation of the part polygon
        float LabelAngle = PartAngle + angleShift;
        for (int t = 0; t < LabelPolygon.size(); t++)
          RotatedPolygon[t] = spelHelper::rotatePoint2D(partPolygon[t], PartCenter, angleShift);
        for (float x = -SearchDist; x < SearchDist; x += minStep)
          for (float y = -SearchDist; y < SearchDist; y += minStep)
          {
            // Shift label polygon
            for (int t = 0; t < LabelPolygon.size(); t++)
              LabelPolygon[t] = RotatedPolygon[t] + cv::Point2f(x, y);
            cv::Point2f LabelCenter = PartCenter + cv::Point2f(x, y);
            PartLabels.push_back(LimbLabel(id, LabelCenter, LabelAngle, LabelPolygon, std::vector<Score>(), false));
          }	  
      }

      // Adding one label if part labels is empty
      if (PartLabels.size() == 0)
      {
        LimbLabel Label(id, getPartCenter(PartRects[id]), spelHelper::getAngle(PartRects[id][0], PartRects[id][1]), PartRects[id], std::vector<Score>(), false); // !?????? 
        PartLabels.push_back(Label);
      }

      Labels.emplace(std::pair<uint32_t, std::vector<LimbLabel>> (id, PartLabels));
      PartLabels.clear();
    }

    return Labels;
  }

  std::map<uint32_t, std::vector<LimbLabel>> SURFDetector2::Detect(Frame* frame_, std::map<uint32_t, std::vector<LimbLabel>> &Labels) const
  {
    uint8_t debugLevel = 2;
    std::string detectorName = std::to_string(getID());

    DebugMessage(" Preparing frame " + std::to_string(frame_->getID()) + "\n", debugLevel);

    Frame* frame = frame_;
    /*bool scalingFrame = (parameters.internalFrameHeight != 0);
    scalingFrame = (scalingFrame && parameters.internalFrameHeight != frame_->getImageSize().height);
    if (scalingFrame)
    {
      frame = preparedFrame(frame_);
      DebugMessage(" Scaling frame \n" , debugLevel);
    }*/

    DebugMessage(" SURFDetector Detect started", debugLevel);
    cv::Mat image = frame->getImage();

/*    float reverseScale = 1.0f;
    if (parameters.externalFrameHeight == 0 && parameters.internalFrameHeight > 0) // !??????
    {	
      int inputFrameHeight = frame_->getImageSize().height;
      reverseScale = static_cast<float>(inputFrameHeight) / static_cast<float>(parameters.internalFrameHeight);
    }
      
    if (parameters.internalFrameHeight > 0 && parameters.externalFrameHeight > 0)
      reverseScale = static_cast<float>(parameters.externalFrameHeight) / static_cast<float>(parameters.internalFrameHeight);
*/
    /*float adjustScale = 1.0f; 
    if(parameters.adjustSolves)
      adjustScale = 1.0f / frame->getScale(); // ! */

    // Calculate frame keypoints
    std::vector<cv::KeyPoint> Keypoints = detectKeypoints(frame, parameters.useMask);
    DebugMessage(" Mask keypoints count: " + std::to_string(Keypoints.size()) + "\n", debugLevel);

    // Extract descriptors
    cv::Mat FrameDescriptors;
    cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    extractor->compute(image, Keypoints, FrameDescriptors);

    // Create skeleton model for the current frame
    //SkeletonModel Local;
    SkeletonModel Temp;
    Local.clear();
    //Local.PartCellsCount = Trained.PartCellsCount;
    Local.Keypoints = Keypoints;
    Local.Descriptors = FrameDescriptors;
    Skeleton skeleton = frame->getSkeleton();
    std::map<int, std::vector<cv::Point2f>> PartRects = getAllPolygons(skeleton);
    for (int i = 0; i < PartRects.size(); i++)
    {
      Local.PartKeypoints.emplace(i, std::vector<int>());
      Temp.PartKeypoints.emplace(i, std::vector<int>());
    }

    // Create matches
    int n = 2; //matches per keypoint
    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(FrameDescriptors, Trained.Descriptors, matches, n);
    DebugMessage(" Matches was created", debugLevel);

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
    DebugMessage(" Matches was normalized", debugLevel);
    
    // select parts keypoints candidates 
    for (unsigned int i = 0; i < matches.size(); i++)
    {
      int t = matches[i][0].trainIdx;
      int c = Trained.Keypoints[t].class_id;
      if (c >= 0)
      {
        int id = PartIndex(c);
        //std::cout << i << ": " << c << "~" << id << std::endl;
        if(id >= 0 && id < Trained.PartCellsCount.size())
          Local.PartKeypoints[id].push_back(i);
        else
          DebugMessage(" Invalid part cell index", debugLevel);
      }
    }

    float K = static_cast<float>(Trained.StudiedFramesID.size()); // Score coefficient

    //std::map<uint32_t, std::vector<LimbLabel>> Labels;
    for (int id = 0; id < PartRects.size(); id++)
    {
      std::string partID = std::to_string(id);
      if (Trained.PartKeypoints.at(id).size() == 0)
        DebugMessage(" Body Part " + partID + " model is empty\n", debugLevel);

      bool negativeScore = false;
      int LabelsPerPart = 0;

      if ((Trained.PartKeypoints.at(id).size() > 0)) // or " > threshold"
      {
        cv::Size CellsCount = Trained.PartCellsCount[id];
        std::vector<int> keypointsCandidates = Local.PartKeypoints.at(id);

        for (int l = 0; l < Labels[id].size(); l++)
        {
          // Calculate score for current label
          double LabelScore = 0;
          std::vector<cv::Point2f> LabelPolygon = Labels[id][l].getPolygon();

          for(unsigned int p = 0; p < keypointsCandidates.size(); p++)
          {
            int k = keypointsCandidates[p];
            if(pointPolygonTest(LabelPolygon, Local.Keypoints[k].pt, false) > 0)
            {
              int partCellID = PartCellIndex(id, Local.Keypoints[k].pt, LabelPolygon, CellsCount);
              if(Trained.Keypoints[matches[k][0].trainIdx].class_id == partCellID)
              {
                LabelScore = LabelScore + matches[k][0].distance;
                Temp.PartKeypoints[id].push_back(k);
              }
            }
          }

          // Normalize and inverse score
          if (LabelScore > 0)
          {
            LabelScore = 1.0 - K*LabelScore / static_cast<double>(Trained.PartKeypoints[id].size());
            LabelsPerPart++;
          }           
          else
            LabelScore = 1.1f; // {-1.0f, INFINITY} !??????
          //Local.PartKeypoints[id].clear();
          if(LabelScore < 0.0f)
          negativeScore = true; 

          // Create score
          Score score(static_cast<float>(LabelScore), detectorName);

          // Push score to LimbLabel scores
          Labels[id][l].addScore(score);

          // Rescale LimpLabel
          /*if (parameters.adjustSolves == true && adjustScale != 0.0f && adjustScale != 1.0f)
             Label.Resize(adjustScale);
           else*/
          /*  if(reverseScale > 0 && reverseScale != 1.0f)
                Label.Resize(reverseScale);*/
        }

        keypointsCandidates.clear();
        sort(Labels[id].begin(), Labels[id].end(), CompareLabels());

      }

      // Renormalize scores
      // Insurance, temporary fix: score can be less then 0.0f if keyframes has too different keypoints combination
      if (negativeScore)
      {
        DebugMessage("Renormalize scores for part " + partID + "\n", debugLevel);

        float scoreValue = 0.0f;
        float minScore = 0.0f;
        for (int l = 0; l < Labels[id].size(); l++)
        {
          scoreValue = Labels[id][l].getAvgScore();
          if(scoreValue < minScore)
            minScore = scoreValue;
        }
        if(minScore < 0.0f)
        {
          for (int l = 0; l < Labels[id].size(); l++)
          {
            scoreValue = (Labels[id][l].getAvgScore() - minScore)/(1.0f - minScore);
            Score score(scoreValue, detectorName);
            std::vector<Score> scores;
            scores.push_back(score);          
            Labels[id][l].setScores(scores);
          }
        }

        std::string message = " Limb Labels count per Body Part " + partID + " found: " + std::to_string(LabelsPerPart) + "\n";
        DebugMessage(message, debugLevel);
      }
    }

    Local = Temp;
    Temp.clear();

    /*if (scalingFrame)
      delete frame;
    else*/
    {
      if (frame->GetImagePath().empty()) frame->cacheImage(); // !??????
      frame->UnloadImage();
      if (frame->GetImagePath().empty()) frame->cacheMask(); // !??????
      frame->UnloadMask();	  
    }
    detectorName.clear();

    DebugMessage(" SURFDetector Detect completed", debugLevel);

    return Labels;
  }

  std::map<uint32_t, std::vector<LimbLabel>> SURFDetector2::detect(Frame* frame,
       std::map<std::string, float> params, 
      std::map<uint32_t, std::vector<LimbLabel>> &limbLabels) const
  {
    changeParameters(params);

    std::map<uint32_t, std::vector<LimbLabel>> NewLabels;
    if (limbLabels.size() == 0)
    {
      Skeleton skeleton = frame->getSkeleton();
      NewLabels = generatePartsLabels(skeleton);
      //NewLabels = generateLimbLabels(frame, params);
    }

    else
      NewLabels = limbLabels;
    NewLabels = SURFDetector2::Detect(frame, NewLabels);

    std::stringstream detectorName;
    detectorName << getID();

    // Apply merge and filter
    /*std::map<uint32_t, std::vector<LimbLabel>> filteredLabels;
    for (uint32_t i = 0; i < NewLabels.size(); i++)
    {
      std::vector<LimbLabel> partLabels = Detector::filterLimbLabels(NewLabels[i],
          parameters.uniqueLocationCandidates, parameters.uniqueAngleCandidates);
      spelHelper::RecalculateScoreIsWeak(partLabels, detectorName.str(),
          parameters.isWeakThreshold);
      filteredLabels.emplace(std::pair<uint32_t, std::vector<LimbLabel>>(i, partLabels));
    }*/

    //return merge(limbLabels, filteredLabels, std::map<uint32_t, std::vector<LimbLabel>>());
    return NewLabels;
  }

  // It is gag
  LimbLabel SURFDetector2::generateLabel(const BodyPart &bodyPart,
      Frame *workFrame, const cv::Point2f &parent, const cv::Point2f &child,
     DetectorHelper *detectorHelper, std::map<std::string, float> params) const
  {
    // Compute frame keypoints, descriptors and matches
    cv::Mat image = workFrame->getImage();
    cv::Mat Mask = workFrame->getMask();
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SurfFeatureDetector::create(parameters.minHessian);
    std::vector<cv::KeyPoint> FrameKeypoints;
    detector->detect(image, FrameKeypoints);
    std::vector<cv::KeyPoint> MaskKeypoins;
    if(parameters.useMask == true)
      MaskKeypoins = SelectMaskKeypoints(Mask, FrameKeypoints);
    else
      MaskKeypoins = FrameKeypoints;

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
    std::vector<cv::Point2f> polygon = buildPartPolygon(bodyPart.getLWRatio(), parent, child);
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

  void SURFDetector2::calculateLabelScore(Frame* workFrame, DetectorHelper* detectorHelper,
      LimbLabel &label, std::map<std::string, float> params) const
  {

  }

  std::vector<cv::KeyPoint> SURFDetector2::getPartKeypoints(int partID) const
  {
    std::vector<cv::KeyPoint> keypoints;

    if (partID < Local.PartKeypoints.size())
      for (int i = 0; i < Local.PartKeypoints[partID].size(); i++)
      {
        int k = Local.PartKeypoints[partID][i];
        keypoints.push_back(Local.Keypoints[k]);
      }
    return keypoints;
  }

  std::map<uint32_t, std::vector<cv::KeyPoint>> SURFDetector2::getPartsKeypoints() const
  {
    std::map<uint32_t, std::vector<cv::KeyPoint>> partsKeytoints;
    for(int i = 0; i < Local.PartKeypoints.size(); i++)
      partsKeytoints.emplace(std::pair<int, std::vector<cv::KeyPoint>>(i, getPartKeypoints(i)));

    return partsKeytoints;
  }
}