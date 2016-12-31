#include "surfDetectorB.hpp"
//#include "surfDetector.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"


namespace SPEL
{
  void SURFDetector::SkeletonModel::clear(void)
  {
    PartCellsCount.clear();
    PartKeypoints.clear();
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

  SURFDetector::SURFDetector(std::map<std::string, float> params)
  {
    m_id = 0x53440000;
    setParameters(params);
  }

  SURFDetector::~SURFDetector(void)
  {
    Trained.clear();
  }

  // The set of parameters which can be changed only before training
  void SURFDetector::setParameters(std::map<std::string, float> params)
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

  Frame* SURFDetector::preparedFrame(Frame* frame) const
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

  void SURFDetector::setAutoInternalFrameHeight(std::vector<Frame*> frames)
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
    if(CellsCount == cv::Size(0,0))
      CellsCount = Trained.PartCellsCount[PartID];
    double L = getLenght(polygon);
    double W = getWidth(polygon);
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

  int SURFDetector::PartIndex(int CellIndex) const
  {
    return static_cast<int>(trunc(float(CellIndex) / float(partCellsLimit)));
  }

  void SURFDetector::setCellsCount(std::map<int, std::vector<cv::Point2f>> &partPolygons, float markingError)
  {
    for (unsigned int k = 0; k < partPolygons.size(); k++)
    {
      cv::Size cellsCount(1, 1);
      if ( abs(markingError) > 1.0f)
      {
        float e = abs(ceil(markingError));
        int lenghtCells = static_cast<int>(trunc(getLenght(partPolygons[k]) / e));
        int widthCells = static_cast<int>(trunc(getWidth(partPolygons[k]) / e));
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

  void SURFDetector::correctingCellsSize(std::map<int, std::vector<cv::Point2f>> &partPolygons)
  {
    for (unsigned int i = 0; i < partPolygons.size(); i++)
    {
      float lenght = getLenght(partPolygons[i]);
      float widht = getWidth(partPolygons[i]);
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
  uchar C_[3][3] = { { 7, 0, 1 },{ 6, 0, 2 },{ 5, 4, 3 } }; // Matrix for search contour operation
  cv::Point2i P_[8] = { cv::Point2i(0, -1), cv::Point2i(1, -1), cv::Point2i(1, 0), cv::Point2i(1, 1), cv::Point2i(0, 1), cv::Point2i(-1, 1), cv::Point2i(-1, 0), cv::Point2i(-1, -1) }; // Matrix for search contour operation

  // Search contour iteration
  bool f_(cv::Mat mask, cv::Point2i &p0, cv::Point2i &p1, cv::Point2i p00)
  {
    int Q = 9;
    cv::Size size = mask.size();
    bool color = 0;
    cv::Point2i d = p0 - p1, p = p0;
    int n = C_[d.y + 1][d.x + 1];

    int  k = 0;
    while (!color)
    { 
      p0 = p;
      p = p1 + P_[n];
      if (k > 7) p = p00;
      if((p.x >= 0) && (p.x < size.width) && (p.y >= 0) && (p.y < size.height))
      { color = (mask.at<uchar>(p.y, p.x) > Q); }
      else { color = 0; }
      n++;
      if (n > 7) n = 0;
      k++;
    }
    p1 = p;
    return (p1 == p00);
  }

  cv::Rect resizeROI_(cv::Rect ROI, cv::Size NewROISize, cv::Size ImageSize)
  {
    float dx = static_cast<float>(NewROISize.width - ROI.width);
    float dy = static_cast<float>(NewROISize.height - ROI.height);
    float x = static_cast<float>(ROI.x);
    float y = static_cast<float>(ROI.y);

    cv::Point2f p0 = cv::Point2f(x, y) - 0.5f*cv::Point2f(dx, dy);

    if (p0.x < 0) p0.x = 0;
    if (p0.y < 0) p0.y = 0;

    if (ImageSize != cv::Size(0, 0))
    {
      if (static_cast<int>(p0.x) + NewROISize.width > ImageSize.width)
        NewROISize.width = ImageSize.width - static_cast<int>(p0.x);
      if (static_cast<int>(p0.y) + NewROISize.height > ImageSize.height)
        NewROISize.height = ImageSize.height - static_cast<int>(p0.y);
    }

    return cv::Rect(p0, NewROISize);
  }

  // Search all ROI on the mask and return coordinates of max ROI: {topLeft, bottomRight}
  cv::Rect SearchROI_(cv::Mat mask)
  {
    cv::Size size = mask.size();
    int cols = size.width;
    int rows = size.height;

    int Q = 10 - 1;
    int maxArea = 0;
    int N = -1;

    std::vector<std::pair<cv::Point2i, cv::Point2i>> ROI;

    for (int y = 0; y < rows; y++)
      for (int x = 0; x < cols; x++)
      {
        cv::Point2i p(x, y);
        bool b = false;
        for (int i = 0; i < ROI.size(); i++)
        {
          if((p.x >= ROI[i].first.x) && (p.x <= ROI[i].second.x) && (p.y >= ROI[i].first.y) && (p.y <= ROI[i].second.y))
          {
            x = ROI[i].second.x;
            b = true;
          }
        }
        if ((!b) && (mask.at<uchar>(y, x) > Q))
        {
          cv::Point2i  p1 = p, p0 = p + cv::Point2i(-1, 0), p00 = p;

          bool FindedStartPoint = false;
          cv::Point2i A(cols, rows), B(0, 0);

          while (!FindedStartPoint)
          {
            FindedStartPoint = f_(mask, p0, p1, p00);

            if (p1.x < A.x) A.x = p1.x;
            if (p1.x > B.x) B.x = p1.x;
            if (p1.y < A.y) A.y = p1.y;
            if (p1.y > B.y) B.y = p1.y;
          }
          ROI.push_back(std::pair<cv::Point2f, cv::Point2f>(A, B));
          cv::Point2f P = B - A;
          float S = P.x*P.y;
          if ( S > maxArea)
          {
            maxArea = static_cast<int>(S);
            N = static_cast<int>(ROI.size()) - 1;
          }
        }		
      }
    std::vector<cv::Point2i> temp;
    if (N > -1)
    {
      temp.push_back(ROI[N].first);
      temp.push_back(ROI[N].second);
    }

    cv::Rect maskROI(temp[0], temp[1]);
    /*cv::Size borderSize = cv::Size(5, 5);
    borderSize += borderSize;
    maskROI = resizeROI_(maskROI, maskROI.size() + borderSize, mask.size());*/

    return maskROI;
  }
  //======================================

  std::vector<cv::KeyPoint> SURFDetector::detectKeypoints(Frame* frame, bool useMask) const
  {
    cv::Mat Image = frame->getImage();

    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SurfFeatureDetector::create(parameters.minHessian);
    std::vector<cv::KeyPoint> Keypoints;
    if (useMask == true)
    {
      // Detect ROI keypoints
      cv::Mat Mask = frame->getMask();
      cv::Rect ROI = SearchROI_(Mask);
      //Image.adjustROI(ROI.y, ROI.y+ROI.height, ROI.x, ROI.width);
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

  void SURFDetector::SingleFrameTrain(Frame* frame_)
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
    /*if(parameters.externalFrameHeight == 0)
    {
      parameters.externalFrameHeight = frames[0]->getImageSize().height; // !??????
      params.at("externalFrameHeight") = parameters.externalFrameHeight; // !??????
    }*/
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

  std::map<uint32_t, std::vector<LimbLabel>> SURFDetector::Detect(Frame* frame_) const
  {
    uint8_t debugLevel = 2;

    Frame* frame = frame_;
    bool scalingFrame = (parameters.internalFrameHeight != 0);
    scalingFrame = (scalingFrame && parameters.internalFrameHeight != frame_->getImageSize().height);
    if (scalingFrame)
    {
      frame = preparedFrame(frame_);
      std::stringstream s;
      s << " Scaling frame " << frame_->getID() << std::endl;
      DebugMessage(s.str(), debugLevel);
      s.clear();
    }

    DebugMessage(" SURFDetector Detect started", debugLevel);
    cv::Mat image = frame->getImage();

    float reverseScale = 1.0f;
    if (parameters.externalFrameHeight == 0 && parameters.internalFrameHeight > 0) // !??????
    {	
      int inputFrameHeight = frame_->getImageSize().height;
      reverseScale = static_cast<float>(inputFrameHeight) / static_cast<float>(parameters.internalFrameHeight);
    }
      
    if (parameters.internalFrameHeight > 0 && parameters.externalFrameHeight > 0)
      reverseScale = static_cast<float>(parameters.externalFrameHeight) / static_cast<float>(parameters.internalFrameHeight);

    /*float adjustScale = 1.0f; 
    if(parameters.adjustSolves)
      adjustScale = 1.0f / frame->getScale(); // ! */

    // Calculate frame keypoints
    std::vector<cv::KeyPoint> Keypoints = detectKeypoints(frame, parameters.useMask);

    std::stringstream s1;
    s1 << " Mask keypoints count: " << Keypoints.size() << std::endl;
    DebugMessage(s1.str(), debugLevel);
    s1.clear();

    // Extract descriptors
    cv::Mat FrameDescriptors;
    cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
    extractor->compute(image, Keypoints, FrameDescriptors);

    // Create skeleton model for the current frame
    SkeletonModel Local;
    Local.Keypoints = Keypoints;
    Local.Descriptors = FrameDescriptors;
    Skeleton skeleton = frame->getSkeleton();
    std::map<int, std::vector<cv::Point2f>> PartRects = getAllPolygons(skeleton);
    for (int i = 0; i < PartRects.size(); i++)
      Local.PartKeypoints.emplace(i, std::vector<int>());

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

    std::map<uint32_t, std::vector<LimbLabel>> Labels;

    for (int id = 0; id < PartRects.size(); id++)
    {
      if(Trained.PartKeypoints.at(id).size() == 0)
      {
        std::stringstream s;
        s << " Body Part " << id << " model is empty" << std::endl;
        DebugMessage(s.str(), debugLevel);
        s.clear();
      }
      if(Trained.PartKeypoints.at(id).size() > 0) // or " > threshold"
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
        std::vector<int> keypointsCandidates = Local.PartKeypoints.at(id);

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

              for(unsigned int p = 0; p < keypointsCandidates.size(); p++)
              {
                int k = keypointsCandidates[p];
                if(pointPolygonTest(LabelPolygon, Local.Keypoints[k].pt, false) > 0)
                {
                  int partCellID = PartCellIndex(id, Local.Keypoints[k].pt, LabelPolygon, CellsCount);
                  if(Trained.Keypoints[matches[k][0].trainIdx].class_id == partCellID)
                  {
                    LabelScore = LabelScore + matches[k][0].distance;
                    //Local.PartKeypoints[id].push_back(p);
                  }
                }
              }
              // Normalize and inverse score
              if(LabelScore > 0)
                LabelScore = 1.0 - LabelScore / static_cast<double>(Trained.PartKeypoints[id].size());
              else
                LabelScore = INFINITY;
              //Local.PartKeypoints[id].clear();

              // Create scores
              Score score(static_cast<float>(LabelScore), "");
              std::vector<Score> scores;
              scores.push_back(score);

              // Rescale polygon points
              /*if (parameters.adjustSolves == true && adjustScale != 0.0f && adjustScale != 1.0f)
              {
                for (int t = 0; t < LabelPolygon.size(); t++)
                  LabelPolygon[t] *= adjustScale;
                LabelCenter *= adjustScale;
              }
              else*/
                if(reverseScale > 0 && reverseScale != 1.0f)
                {
                  for (int t = 0; t < LabelPolygon.size(); t++)
                    LabelPolygon[t] *= reverseScale;
                  LabelCenter *= reverseScale;
                }
              
              // Create LimbLabel
              LimbLabel Label(id, LabelCenter, LabelAngle, LabelPolygon, scores);
              PartLabels.push_back(Label);
              scores.clear();
            }
        }
        keypointsCandidates.clear();
        sort(PartLabels.begin(), PartLabels.end(), CompareLabels());
        Labels.emplace(std::pair<int, std::vector<LimbLabel>>(id, PartLabels));
        PartLabels.clear();
      
        std::stringstream s;
        s << " Limb Labels count per Body Part " << id <<": " << LabelsPerPart << std::endl;
        DebugMessage(s.str(), debugLevel);
        s.clear();
      }
    }

    if (scalingFrame)
      delete frame;
    else
    {
      if (frame->GetImagePath().empty()) frame->cacheImage(); // !??????
      frame->UnloadImage();
      if (frame->GetImagePath().empty()) frame->cacheMask(); // !??????
      frame->UnloadMask();	  
    }

    DebugMessage(" SURFDetector Detect completed", debugLevel);
    return Labels;
  }

  std::map<uint32_t, std::vector<LimbLabel>> SURFDetector::detect(Frame* frame,
       std::map<std::string, float> params, 
      const std::map<uint32_t, std::vector<LimbLabel>> &limbLabels) const
  {
    changeParameters(params);
    std::map<uint32_t, std::vector<LimbLabel>> NewLabels;
    NewLabels = SURFDetector::Detect(frame);

    std::stringstream detectorName;
    detectorName << getID();

    // Apply merge and filter
    std::map<uint32_t, std::vector<LimbLabel>> filteredLabels;
    for (uint32_t i = 0; i < NewLabels.size(); i++)
    {
      std::vector<LimbLabel> partLabels = Detector::filterLimbLabels(NewLabels[i],
          parameters.uniqueLocationCandidates, parameters.uniqueAngleCandidates);
      spelHelper::RecalculateScoreIsWeak(partLabels, detectorName.str(),
          parameters.isWeakThreshold);
      filteredLabels.emplace(std::pair<uint32_t, std::vector<LimbLabel>>(i, partLabels));
    }

    detectorName.clear();

    //return NewLabels;
    return merge(limbLabels, filteredLabels, std::map<uint32_t, std::vector<LimbLabel>>());

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