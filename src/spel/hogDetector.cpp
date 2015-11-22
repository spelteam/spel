#include "hogDetector.hpp"

namespace SPEL
{
  HogDetector::HogDetector(void) noexcept
  {
    id = 0x48440000;
  }

  HogDetector::~HogDetector(void) noexcept
  {
    for (auto &p : partModels)
      for (auto &pp : p.second)
        pp.second.partImage.release();
  }

  HogDetector::PartModel HogDetector::computeDescriptors(const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1, const cv::Mat &imgMat, const int nbins, const cv::Size &wndSize, const cv::Size &blockSize, const cv::Size &blockStride, const cv::Size &cellSize, const double wndSigma, const double thresholdL2hys, const bool gammaCorrection, const int nlevels, const int derivAperture, const int histogramNormType, const bool bGrayImages) const
  {
    auto boneLength = getBoneLength(j0, j1);
    if (boneLength < blockSize.width)
      boneLength = static_cast <float> (blockSize.width);
    else
      boneLength = boneLength + blockSize.width - (static_cast<int>(boneLength) % blockSize.width);

    auto boneWidth = getBoneWidth(boneLength, bodyPart);
    if (boneWidth < blockSize.height)
      boneWidth = static_cast <float> (blockSize.height);
    else
      boneWidth = boneWidth + blockSize.width - (static_cast<int>(boneWidth) % blockSize.height);

    auto originalSize = cv::Size(static_cast <uint32_t> (boneLength), static_cast <uint32_t> (boneWidth));
    auto rect = getBodyPartRect(bodyPart, j0, j1, blockSize);

    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    auto direction = j1 - j0;
    auto rotationAngle = static_cast<float>(spelHelper::angle2D(1.0f, 0.0f, direction.x, direction.y) * (180.0 / M_PI));
    PartModel partModel;
    partModel.partModelRect = rect;
    auto partImage = rotateImageToDefault(imgMat, partModel.partModelRect, rotationAngle, originalSize);
    auto partImageResized = cv::Mat(wndSize.height, wndSize.width, CV_8UC3, cv::Scalar(255, 255, 255));
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
      partModel.partImage = partImageResized.clone();

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
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
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
                  counter.at(r / cellSize.height).at(c / cellSize.width)++;
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
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }

    try
    {
      for (auto i = 0U; i < wndSize.height; i += cellSize.height)
      {
        for (auto j = 0U; j < wndSize.width; j += cellSize.width)
        {
          for (auto b = 0; b < nbins; b++)
          {
            if (counter.at(i / cellSize.height).at(j / cellSize.width) == 0)
              partModel.gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).at(b) = 0;
            else
              partModel.gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).at(b) /= (static_cast<float>(counter.at(i / cellSize.height).at(j / cellSize.width)));
          }
        }
      }
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get gradientStrengths at [" << i / cellSize.height << "][" << j / cellSize.width << "]";
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }

    return partModel;
  }

  std::map <uint32_t, HogDetector::PartModel> HogDetector::computeDescriptors(const Frame *frame, const int nbins, const cv::Size &blockSize, const cv::Size &blockStride, const cv::Size &cellSize, const double wndSigma, const double thresholdL2hys, const bool gammaCorrection, const int nlevels, const int derivAperture, const int histogramNormType, const bool bGrayImages) const
  {
    std::map <uint32_t, PartModel> parts;
    cv::Size wndSize;
    auto skeleton = frame->getSkeleton();
    auto partTree = skeleton.getPartTree();
    auto imgMat = frame->getImage();
    for (const auto &part : partTree)
    {
      try
      {
        wndSize = partSize.at(part.getPartID());
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Couldn't get partSize for part " << part.getPartID();
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }
      auto *joint = skeleton.getBodyJoint(part.getParentJoint());
      if (joint == 0)
      {
        const std::string str = "Invalid parent joint";
        DebugMessage(str, 1);
        throw std::logic_error(str);
      }
      auto j0 = joint->getImageLocation();
      joint = 0;
      joint = skeleton.getBodyJoint(part.getChildJoint());
      if (joint == 0)
      {
        const std::string str = "Invalid child joint";
        DebugMessage(str, 1);
        throw std::logic_error(str);
      }
      auto j1 = joint->getImageLocation();
      try
      {
        parts.insert(std::pair <uint32_t, PartModel>(part.getPartID(), computeDescriptors(part, j0, j1, imgMat, nbins, wndSize, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType, bGrayImages)));
      }
      catch (std::logic_error err)
      {
        std::stringstream ss;
        ss << "Can't compute descriptors for the frame " << frame->getID() << " for the part " << part.getPartID() << std::endl;
        ss << "\t" << err.what();
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }
    }
    return parts;
  }

  std::map <uint32_t, cv::Size> HogDetector::getMaxBodyPartHeightWidth(std::vector <Frame*> frames, cv::Size blockSize, float resizeFactor) const
  {
    std::map <uint32_t, cv::Size> result;
    for (const auto frame : frames)
    {
      if (frame->getFrametype() != KEYFRAME && frame->getFrametype() != LOCKFRAME)
        continue;

      auto skeleton = frame->getSkeleton();
      auto bodyParts = skeleton.getPartTree();
      for (const auto &bodyPart : bodyParts)
      {
        auto joint = skeleton.getBodyJoint(bodyPart.getParentJoint());
        if (joint == 0)
        {
          const std::string str = "Invalid parent joint";
          DebugMessage(str, 1);
          throw std::logic_error(str);
        }
        auto j0 = joint->getImageLocation();
        joint = 0;
        joint = skeleton.getBodyJoint(bodyPart.getChildJoint());
        if (joint == 0)
        {
          const std::string str = "Invalid child joint";
          DebugMessage(str, 1);
          throw std::logic_error(str);
        }
        auto j1 = joint->getImageLocation();
        auto boneLength = getBoneLength(j0, j1);
        //TODO (Vitaliy Koshura): Check this!        
        auto boneWidth = getBoneWidth(boneLength, bodyPart);

        auto maxSize = cv::Size(static_cast <uint32_t> (boneLength * resizeFactor), static_cast <uint32_t> (boneWidth * resizeFactor));
        if (result.size() > 0)
        {
          try
          {
            maxSize = result.at(bodyPart.getPartID());
          }
          catch (...) {}
        }
        result[bodyPart.getPartID()] = cv::Size(std::max(maxSize.width, static_cast <int> (boneLength * resizeFactor)), std::max(maxSize.height, static_cast <int> (boneWidth * resizeFactor)));
      }
    }
    // normalize
    for (auto &part : result)
    {
      part.second.width += (blockSize.width - part.second.width % blockSize.width);
      part.second.height += (blockSize.height - part.second.height % blockSize.height);
    }
    return result;
  }

  void HogDetector::train(const std::vector <Frame*> &_frames, std::map <std::string, float> params)
  {
    frames = _frames;

    partSize.clear();
    partModels.clear();

    params.emplace(COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES());
    auto bGrayImages = params.at(COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES().first) != 0.0f;

    params.emplace(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT());
    maxFrameHeight = params.at(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().first);

    auto bFirstConversion = true;
    for (const auto frameNum : frames)
    {
      if (frameNum->getFrametype() != KEYFRAME && frameNum->getFrametype() != LOCKFRAME)
        continue;

      auto originalSize = frameNum->getFrameSize().height;

      Frame *workFrame = 0;
      if (frameNum->getFrametype() == KEYFRAME)
        workFrame = new Keyframe();
      else if (frameNum->getFrametype() == LOCKFRAME)
        workFrame = new Lockframe();
      else if (frameNum->getFrametype() == INTERPOLATIONFRAME)
        workFrame = new Interpolation();

      workFrame = frameNum->clone(workFrame);

      auto scale = workFrame->Resize(maxFrameHeight);

      if (bFirstConversion)
      {
        partSize = getMaxBodyPartHeightWidth(_frames, blockSize, scale);
        bFirstConversion = false;
      }

      if (SpelObject::getDebugLevel() >= 2)
        std::cout << "Training on frame " << workFrame->getID() << std::endl;

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

  std::map <uint32_t, std::vector <LimbLabel> > HogDetector::detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const
  {
    auto detectorHelper = new HogDetectorHelper();

    params.emplace(COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR());
    params.emplace(COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES());

    auto result = Detector::detect(frame, params, limbLabels, detectorHelper);

    delete detectorHelper;

    return result;
  }

  LimbLabel HogDetector::generateLabel(const BodyPart &bodyPart, const Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1, DetectorHelper *detectorHelper, std::map <std::string, float> params) const
  {
    std::stringstream detectorName;
    detectorName << getID();

    params.emplace(COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR());
    params.emplace(COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES());

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
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }

    auto generatedPartModel = computeDescriptors(bodyPart, j0, j1, frame->getImage(), nbins, size, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType, bGrayImages);

    auto comparer = [&]() -> float
    {
      return compare(bodyPart, generatedPartModel, nbins);
    };

    return Detector::generateLabel(bodyPart, j0, j1, detectorName.str(), useHoGdet, comparer);
  }

  float HogDetector::compare(const BodyPart &bodyPart, const PartModel &model, const uint8_t nbins) const
  {
    auto score = 0.0f;
    auto count = 0.0f;
    for (const auto &framePartModels : partModels)
    {
      PartModel partModel;
      try
      {
        partModel = framePartModels.second.at(bodyPart.getPartID());
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Can't find part model for body part " << bodyPart.getPartID();
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }
      if (model.gradientStrengths.size() != partModel.gradientStrengths.size())
      {
        std::stringstream ss;
        ss << "Invalid descriptor count. Need: " << model.gradientStrengths.size() << ". Have: " << partModel.gradientStrengths.size();
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
      for (auto i = 0U; i < model.gradientStrengths.size(); i++)
      {
        if (model.gradientStrengths.at(i).size() != partModel.gradientStrengths.at(i).size())
        {
          std::stringstream ss;
          ss << "Invalid descriptor count. Need: " << model.gradientStrengths.at(i).size() << ". Have: " << partModel.gradientStrengths.at(i).size();
          DebugMessage(ss.str(), 1);
          throw std::logic_error(ss.str());
        }
        for (auto j = 0U; j < model.gradientStrengths.at(i).size(); j++)
        {
          for (auto b = 0; b < nbins; b++)
          {
            try
            {
              count++;
              score += std::abs(model.gradientStrengths.at(i).at(j).at(b) - partModel.gradientStrengths.at(i).at(j).at(b));
            }
            catch (...)
            {
              std::stringstream ss;
              ss << "Can't get some descriptor at [" << i << "][" << j << "][" << b << "]";
              DebugMessage(ss.str(), 1);
              throw std::out_of_range(ss.str());
            }
          }
        }
      }
    }
    return (score / count);
  }

  std::map <uint32_t, std::map <uint32_t, HogDetector::PartModel>> HogDetector::getPartModels(void) const noexcept
  {
    return partModels;
  }

  cv::Size HogDetector::getCellSize(void) const noexcept
  {
    return cellSize;
  }

  uint8_t HogDetector::getnbins(void) const noexcept
  {
    return nbins;
  }

  HogDetectorHelper::HogDetectorHelper(void) noexcept
  {
  }

  HogDetectorHelper::~HogDetectorHelper(void) noexcept
  {
  }

}
