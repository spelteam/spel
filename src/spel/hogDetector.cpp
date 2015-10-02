#include "hogDetector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "

namespace SPEL
{
  HogDetector::HogDetector(void)
  {
    id = 0x4844;
  }

  HogDetector::~HogDetector(void)
  {
    for (auto &&p : partModels)
      for (auto &&pp : p.second)
        pp.second.partImage.release();

    for (auto &&p : labelModels)
      for (auto &&pp : p.second)
        for (auto &&ppp : pp.second)
          ppp.partImage.release();
  }

  int HogDetector::getID(void) const
  {
    return id;
  }

  void HogDetector::setID(int _id)
  {
    id = _id;
  }

  HogDetector::PartModel HogDetector::computeDescriptors(BodyPart bodyPart, cv::Point2f j0, cv::Point2f j1, cv::Mat imgMat, int nbins, cv::Size wndSize, cv::Size blockSize, cv::Size blockStride, cv::Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType, bool bGrayImages)
  {
    float boneLength = getBoneLength(j0, j1);
    if (boneLength < blockSize.width)
    {
      boneLength = static_cast <float> (blockSize.width);
    }
    else
    {
      boneLength = boneLength + blockSize.width - ((int)boneLength % blockSize.width);
    }
    float boneWidth = getBoneWidth(boneLength, bodyPart);
    if (boneWidth < blockSize.height)
    {
      boneWidth = static_cast <float> (blockSize.height);
    }
    else
    {
      boneWidth = boneWidth + blockSize.width - ((int)boneWidth % blockSize.height);
    }
    cv::Size originalSize = cv::Size(static_cast <uint32_t> (boneLength), static_cast <uint32_t> (boneWidth));
    POSERECT <cv::Point2f> rect = getBodyPartRect(bodyPart, j0, j1, blockSize);

    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    cv::Point2f direction = j1 - j0;
    float rotationAngle = float(spelHelper::angle2D(1.0f, 0.0f, direction.x, direction.y) * (180.0 / M_PI));
    PartModel partModel;
    partModel.partModelRect = rect;
    cv::Mat partImage = rotateImageToDefault(imgMat, partModel.partModelRect, rotationAngle, originalSize);
    cv::Mat partImageResized = cv::Mat(wndSize.height, wndSize.width, CV_8UC3, cv::Scalar(255, 255, 255));
    resize(partImage, partImageResized, wndSize);
    if (bGrayImages)
    {
#if OpenCV_VERSION_MAJOR == 2
      cvtColor(partImageResized, partModel.partImage, CV_BGR2GRAY);
#elif OpenCV_VERSION_MAJOR >= 3
      cvtColor(partImageResized, partModel.partImage, cv::COLOR_BGR2GRAY);
#else
#error "Unsupported version of OpenCV"
#endif
    }
    else
    {
      partModel.partImage = partImageResized.clone();
    }
    cv::HOGDescriptor detector(wndSize, blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);

    std::vector <float> descriptors;

    detector.compute(partModel.partImage, descriptors);

#ifdef DEBUG
    partModel.descriptors = descriptors;
#endif  // DEBUG

    std::vector <std::vector <uint32_t>> counter;

    uint32_t i, j, b;

    try
    {
      for (i = 0; i < wndSize.height; i += cellSize.height)
      {
        partModel.gradientStrengths.push_back(std::vector <std::vector <float>>());
        counter.push_back(std::vector <uint32_t>());
        for (j = 0; j < wndSize.width; j += cellSize.width)
        {
          partModel.gradientStrengths.at(i / cellSize.height).push_back(std::vector <float>());
          counter.at(i / cellSize.height).push_back(0);
          for (b = 0; b < nbins; b++)
          {
            partModel.gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).push_back(0.0f);
          }
        }
      }
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get gradientStrengths at [" << i / cellSize.height << "][" << j / cellSize.width << "]";
#ifdef DEBUG
      std::cerr << ERROR_HEADER << ss.str() << std::endl;
#endif // DEBUG
      throw std::logic_error(ss.str());
    }

    uint32_t d = 0, n, k, r, c;
    try
    {
      // window rows
      for (n = 0; n + blockStride.height < wndSize.height; n += blockStride.height)
      {
        // window cols
        for (k = 0; k + blockStride.width < wndSize.width; k += blockStride.width)
        {
          // block rows
          for (r = n; r < n + blockSize.height; r += cellSize.height)
          {
            // block cols
            for (c = k; c < k + blockSize.width; c += cellSize.width)
            {
              // nbins
              for (b = 0; b < nbins; b++)
              {
                partModel.gradientStrengths.at(r / cellSize.height).at(c / cellSize.width).at(b) += descriptors.at(d);
                if (b == 0)
                {
                  counter.at(r / cellSize.height).at(c / cellSize.width)++;
                }
                d++;
              }
            }
          }
        }
      }
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Descriptor parse error:" << std::endl << "Window row:\t" << n << "\tWindow col:\t" << k << std::endl << "Block row:\t" << r << "\tBlock col:\t" << c << std::endl << "NBins:\t" << b << std::endl;
      ss << "Total image rows:\t" << wndSize.height << "\tTotal image cols:\t" << wndSize.width << std::endl;
      ss << "Total descriptors:\t" << descriptors.size() << std::endl;
      ss << "Trying to get descriptor at:\t" << d << std::endl;
#ifdef DEBUG
      std::cerr << ERROR_HEADER << ss.str() << std::endl;
#endif // DEBUG
      throw std::logic_error(ss.str());
    }

    try
    {
      for (uint32_t i = 0; i < wndSize.height; i += cellSize.height)
      {
        for (uint32_t j = 0; j < wndSize.width; j += cellSize.width)
        {
          for (uint8_t b = 0; b < nbins; b++)
          {
            if (counter.at(i / cellSize.height).at(j / cellSize.width) == 0)
            {
              partModel.gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).at(b) = 0;
            }
            else
            {
              partModel.gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).at(b) /= ((float)counter.at(i / cellSize.height).at(j / cellSize.width));
            }
          }
        }
      }
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get gradientStrengths at [" << i / cellSize.height << "][" << j / cellSize.width << "]";
#ifdef DEBUG
      std::cerr << ERROR_HEADER << ss.str() << std::endl;
#endif // DEBUG
      throw std::logic_error(ss.str());
    }

    return partModel;
  }

  std::map <uint32_t, HogDetector::PartModel> HogDetector::computeDescriptors(Frame *frame, int nbins, cv::Size blockSize, cv::Size blockStride, cv::Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType, bool bGrayImages)
  {
    std::map <uint32_t, PartModel> parts;
    cv::Size wndSize;
    Skeleton skeleton = frame->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    cv::Mat imgMat = frame->getImage();
    for (tree <BodyPart>::iterator part = partTree.begin(); part != partTree.end(); ++part)
    {
      try
      {
        wndSize = partSize.at(part->getPartID());
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Couldn't get partSize for part " << part->getPartID();
        if (debugLevelParam >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }
      cv::Point2f j0, j1;
      BodyJoint *joint = skeleton.getBodyJoint(part->getParentJoint());
      if (joint == 0)
      {
        std::stringstream ss;
        ss << "Invalid parent joint";
        if (debugLevelParam >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }
      j0 = joint->getImageLocation();
      joint = 0;
      joint = skeleton.getBodyJoint(part->getChildJoint());
      if (joint == 0)
      {
        std::stringstream ss;
        ss << "Invalid child joint";
        if (debugLevelParam >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }
      j1 = joint->getImageLocation();
      cv::Point2f direction = j1 - j0; // used as estimation of the vector's direction
      float rotationAngle = float(spelHelper::angle2D(1.0f, 0.0f, direction.x, direction.y) * (180.0 / M_PI)); //bodypart tilt angle 
      part->setRotationSearchRange(rotationAngle);
      try
      {
        parts.insert(std::pair <uint32_t, PartModel>(part->getPartID(), computeDescriptors(*part, j0, j1, imgMat, nbins, wndSize, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType, bGrayImages)));
      }
      catch (std::logic_error err)
      {
        std::stringstream ss;
        ss << "Can't compute descriptors for the frame " << frame->getID() << " for the part " << part->getPartID() << std::endl;
        ss << "\t" << err.what();
        if (debugLevelParam >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }
    }
    skeleton.setPartTree(partTree);
    frame->setSkeleton(skeleton);
    return parts;
  }

  std::map <uint32_t, cv::Size> HogDetector::getMaxBodyPartHeightWidth(std::vector <Frame*> frames, cv::Size blockSize, float resizeFactor)
  {
    std::map <uint32_t, cv::Size> result;
    for (std::vector <Frame*>::iterator frame = frames.begin(); frame != frames.end(); ++frame)
    {
      if ((*frame)->getFrametype() != KEYFRAME && (*frame)->getFrametype() != LOCKFRAME)
      {
        continue;
      }
      Skeleton skeleton = (*frame)->getSkeleton();
      tree <BodyPart> bodyParts = skeleton.getPartTree();
      for (tree <BodyPart>::iterator bodyPart = bodyParts.begin(); bodyPart != bodyParts.end(); ++bodyPart)
      {
        cv::Point2f j0, j1;
        BodyJoint *joint = skeleton.getBodyJoint(bodyPart->getParentJoint());
        if (joint == 0)
        {
          if (debugLevelParam >= 1)
            std::cerr << ERROR_HEADER << "Invalid parent joint" << std::endl;
          break;
        }
        j0 = joint->getImageLocation();
        joint = 0;
        joint = skeleton.getBodyJoint(bodyPart->getChildJoint());
        if (joint == 0)
        {
          if (debugLevelParam >= 1)
            std::cerr << ERROR_HEADER << "Invalid child joint" << std::endl;
          break;
        }
        j1 = joint->getImageLocation();
        float boneLength = getBoneLength(j0, j1);
        //TODO (Vitaliy Koshura): Check this!
        float boneWidth = 0;
        boneWidth = getBoneWidth(boneLength, *bodyPart);

        cv::Size maxSize = cv::Size(static_cast <uint32_t> (boneLength * resizeFactor), static_cast <uint32_t> (boneWidth * resizeFactor));
        if (result.size() > 0)
        {
          try
          {
            maxSize = result.at(bodyPart->getPartID());
          }
          catch (...){}
        }
        result[bodyPart->getPartID()] = cv::Size(std::max(maxSize.width, static_cast <int> (boneLength * resizeFactor)), std::max(maxSize.height, static_cast <int> (boneWidth * resizeFactor)));
      }
    }
    // normalize
    for (std::map <uint32_t, cv::Size>::iterator part = result.begin(); part != result.end(); ++part)
    {
      part->second.width += (blockSize.width - part->second.width % blockSize.width);
      part->second.height += (blockSize.height - part->second.height % blockSize.height);
    }
    return result;
  }

  void HogDetector::train(const std::vector <Frame*> &_frames, std::map <std::string, float> params)
  {
    frames = _frames;

    partSize.clear();
    partModels.clear();
    labelModels.clear();

    params.emplace(COMMON_SPEL_PARAMETERS::DEBUG_LEVEL());
    debugLevelParam = static_cast <uint8_t> (params.at(COMMON_SPEL_PARAMETERS::DEBUG_LEVEL().first));

    params.emplace(COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES());
    auto bGrayImages = params.at(COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES().first) != 0.0f;

    params.emplace(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT());
    maxFrameHeight = params.at(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().first);

    bool bFirstConversion = true;
    for (std::vector <Frame*>::iterator frameNum = frames.begin(); frameNum != frames.end(); ++frameNum)
    {
      if ((*frameNum)->getFrametype() != KEYFRAME && (*frameNum)->getFrametype() != LOCKFRAME)
      {
        continue;
      }

      int originalSize = (*frameNum)->getFrameSize().height;

      Frame *workFrame = 0;
      if ((*frameNum)->getFrametype() == KEYFRAME)
        workFrame = new Keyframe();
      else if ((*frameNum)->getFrametype() == LOCKFRAME)
        workFrame = new Lockframe();
      else if ((*frameNum)->getFrametype() == INTERPOLATIONFRAME)
        workFrame = new Interpolation();

      workFrame = (*frameNum)->clone(workFrame);

      float scale = workFrame->Resize(maxFrameHeight);

      if (bFirstConversion)
      {
        partSize = getMaxBodyPartHeightWidth(_frames, blockSize, scale);
        bFirstConversion = false;
      }

      if (debugLevelParam >= 2)
        std::cerr << "Training on frame " << workFrame->getID() << std::endl;

      try
      {
        partModels.insert(std::pair <uint32_t, std::map <uint32_t, PartModel>>(workFrame->getID(), computeDescriptors(workFrame, nbins, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType, bGrayImages)));
      }
      catch (...)
      {
        break;
      }

      delete workFrame;
    }
  }

  std::map <uint32_t, std::vector <LimbLabel> > HogDetector::detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels)
  {
    auto detectorHelper = new HogDetectorHelper();

    params.emplace(COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR());
    params.emplace(COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES());

    auto result = Detector::detect(frame, params, limbLabels, detectorHelper);

    delete detectorHelper;

    return result;
  }

  LimbLabel HogDetector::generateLabel(const BodyPart &bodyPart, const Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1, DetectorHelper *detectorHelper, std::map <std::string, float> params)
  {
    std::stringstream detectorName;
    detectorName << getID();

    auto useHoGdet = params.at(COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR().first);
    auto bGrayImages = params.at(COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES().first) != 0.0f;

    cv::Size size;
    try
    {
      size = partSize.at(bodyPart.getPartID());
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get partSize for body part " << bodyPart.getPartID();
      if (debugLevelParam >= 1)
      {
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }
    }

    PartModel generatedPartModel = computeDescriptors(bodyPart, j0, j1, frame->getImage(), nbins, size, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType, bGrayImages);

    auto comparer = [&]() -> float 
    {
      return compare(bodyPart, generatedPartModel, nbins);
    };

    LimbLabel label = Detector::generateLabel(bodyPart, j0, j1, detectorName.str(), useHoGdet, comparer);

    labelModels[frame->getID()][bodyPart.getPartID()].push_back(generatedPartModel);

    return label;
  }

  float HogDetector::compare(BodyPart bodyPart, PartModel model, uint8_t nbins)
  {
    float score = 0;
    uint32_t totalcount = 0;
    for (std::map <uint32_t, std::map <uint32_t, PartModel>>::iterator framePartModels = partModels.begin(); framePartModels != partModels.end(); ++framePartModels)
    {
      for (std::map <uint32_t, PartModel>::iterator partModel = framePartModels->second.begin(); partModel != framePartModels->second.end(); ++partModel)
      {
        if (partModel->first != static_cast <uint32_t> (bodyPart.getPartID()))
        {
          continue;
        }
        else
        {
          if (model.gradientStrengths.size() != partModel->second.gradientStrengths.size())
          {
            std::stringstream ss;
            ss << "Invalid descriptor count. Need: " << model.gradientStrengths.size() << ". Have: " << partModel->second.gradientStrengths.size();
            if (debugLevelParam >= 1)
              std::cerr << ERROR_HEADER << ss.str() << std::endl;
            throw std::logic_error(ss.str());
          }
          uint32_t count = 0;
          for (uint32_t i = 0; i < model.gradientStrengths.size(); i++)
          {
            if (model.gradientStrengths.at(i).size() != partModel->second.gradientStrengths.at(i).size())
            {
              std::stringstream ss;
              ss << "Invalid descriptor count. Need: " << model.gradientStrengths.at(i).size() << ". Have: " << partModel->second.gradientStrengths.at(i).size();
              if (debugLevelParam >= 1)
                std::cerr << ERROR_HEADER << ss.str() << std::endl;
              throw std::logic_error(ss.str());
            }
            for (uint32_t j = 0; j < model.gradientStrengths.at(i).size(); j++)
            {
              for (uint8_t b = 0; b < nbins; b++)
              {
                try
                {
                  count++;
                  score += abs(model.gradientStrengths.at(i).at(j).at(b) - partModel->second.gradientStrengths.at(i).at(j).at(b));
                }
                catch (...)
                {
                  std::stringstream ss;
                  ss << "Can't get some descriptor at [" << i << "][" << j << "][" << b << "]";
                  if (debugLevelParam >= 1)
                    std::cerr << ERROR_HEADER << ss.str() << std::endl;
                  throw std::logic_error(ss.str());
                }
              }
            }
          }
          totalcount += count;
          break;
        }
      }
    }
    return (score / (float)totalcount);
  }

  std::map <uint32_t, std::map <uint32_t, std::vector <HogDetector::PartModel>>> HogDetector::getLabelModels(void)
  {
    return labelModels;
  }

  std::map <uint32_t, std::map <uint32_t, HogDetector::PartModel>> HogDetector::getPartModels(void)
  {
    return partModels;
  }

  cv::Size HogDetector::getCellSize(void)
  {
    return cellSize;
  }

  uint8_t HogDetector::getnbins(void)
  {
    return nbins;
  }

  HogDetectorHelper::HogDetectorHelper(void)
  {
  }

  HogDetectorHelper::~HogDetectorHelper(void)
  {
  }

}
