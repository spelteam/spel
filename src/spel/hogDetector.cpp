// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
#include "hogDetector.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"
#include "spelParameters.hpp"

namespace SPEL
{
  HogDetector::HogDetector(uint8_t nbins, cv::Size wndStride,
    cv::Size padding, cv::Size blockSize, cv::Size blockStride,
    cv::Size cellSize, double wndSigma, double thresholdL2hys,
    bool gammaCorrection, int nlevels, int derivAperture,
    int histogramNormType)  :
  m_nbins(nbins),
    m_wndStride(wndStride),
    m_padding(padding),
    m_blockSize(blockSize),
    m_blockStride(blockStride),
    m_cellSize(cellSize),
    m_wndSigma(wndSigma),
    m_thresholdL2hys(thresholdL2hys),
    m_gammaCorrection(gammaCorrection),
    m_nlevels(nlevels),
    m_derivAperture(derivAperture),
    m_histogramNormType(histogramNormType)
  {
    m_id = 0x48440000;
  }

  HogDetector::~HogDetector(void) 
  {
    for (auto &p : m_partModels)
      for (auto &pp : p.second)
        pp.second.partImage.release();
  }

  HogDetector::PartModel HogDetector::computeDescriptors(
    const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1,
    const cv::Mat &imgMat, const cv::Size &wndSize) const
  {
    auto boneLength = BodyPart::getBoneLength(j0, j1);
    if (boneLength < m_blockSize.width)
      boneLength = static_cast <float> (m_blockSize.width);
    else
      boneLength = boneLength + m_blockSize.width - (
        static_cast<int>(boneLength) % m_blockSize.width);

    auto boneWidth = bodyPart.getBoneWidth(boneLength);
    if (boneWidth < m_blockSize.height)
      boneWidth = static_cast <float> (m_blockSize.height);
    else
      boneWidth = boneWidth + m_blockSize.width - (
        static_cast<int>(boneWidth) % m_blockSize.height);

    const auto &originalSize = cv::Size(static_cast <uint32_t> (boneLength),
      static_cast <uint32_t> (boneWidth));
    const auto &rect = bodyPart.getBodyPartRect(j0, j1, m_blockSize);

    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    const auto &direction = j1 - j0;
    const auto rotationAngle = spelHelper::getAngle(direction);
    PartModel partModel;
    partModel.partModelRect = rect;
    auto partImage = spelHelper::rotateImageToDefault(imgMat,
      partModel.partModelRect, rotationAngle, originalSize);
    auto partImageResized = cv::Mat(wndSize.height, wndSize.width, CV_8UC3,
      cv::Scalar(255, 255, 255));
    resize(partImage, partImageResized, wndSize);
    if (m_bGrayImages)
      cvtColor(partImageResized, partModel.partImage, cv::COLOR_BGR2GRAY);
    else
      partModel.partImage = partImageResized.clone();

    std::vector <float> descriptors;
    partModel.gradientStrengths = HogDetector::calculateHog(
      partModel.partImage, descriptors, wndSize, m_blockSize, m_blockStride,
      m_cellSize, m_nbins, m_derivAperture, m_wndSigma, m_histogramNormType,
      m_thresholdL2hys, m_gammaCorrection, m_nlevels);
#ifdef DEBUG
    partModel.descriptors = descriptors;
#endif  // DEBUG

    return partModel;
  }

  std::map <uint32_t, HogDetector::PartModel> HogDetector::computeDescriptors(
    Frame *frame) const
  {
    std::map <uint32_t, PartModel> parts;
    cv::Size wndSize;
    const auto &skeleton = frame->getSkeleton();
    const auto &partTree = skeleton.getPartTree();
    for (const auto &part : partTree)
    {
      try
      {
        wndSize = m_partSize.at(part.getPartID());
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
      const auto &j0 = joint->getImageLocation();
      joint = skeleton.getBodyJoint(part.getChildJoint());
      if (joint == 0)
      {
        const std::string str = "Invalid child joint";
        DebugMessage(str, 1);
        throw std::logic_error(str);
      }
      const auto &j1 = joint->getImageLocation();
      try
      {
        parts.insert(std::pair <uint32_t, PartModel>(part.getPartID(),
          computeDescriptors(part, j0, j1, frame->getImage(), wndSize)));
      }
      catch (std::exception &err)
      {
        std::stringstream ss;
        ss << "Can't compute descriptors for the frame " << frame->getID() <<
          " for the part " << part.getPartID() << std::endl;
        ss << "\t" << err.what();
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }
    }
    return parts;
  }

  std::map <uint32_t, cv::Size> HogDetector::getMaxBodyPartHeightWidth(
    const cv::Size &blockSize, const float resizeFactor) const
  {
    std::map <uint32_t, cv::Size> result;
    for (const auto &frame : m_frames)
    {
      if (frame->getFrametype() != KEYFRAME &&
        frame->getFrametype() != LOCKFRAME)
        continue;

      const auto &skeleton = frame->getSkeleton();
      const auto &bodyParts = skeleton.getPartTree();
      for (const auto &bodyPart : bodyParts)
      {
        auto joint = skeleton.getBodyJoint(bodyPart.getParentJoint());
        if (joint == 0)
        {
          const std::string str = "Invalid parent joint";
          DebugMessage(str, 1);
          throw std::logic_error(str);
        }
        const auto &j0 = joint->getImageLocation();
        joint = skeleton.getBodyJoint(bodyPart.getChildJoint());
        if (joint == 0)
        {
          const std::string str = "Invalid child joint";
          DebugMessage(str, 1);
          throw std::logic_error(str);
        }
        const auto &j1 = joint->getImageLocation();
        const auto boneLength = BodyPart::getBoneLength(j0, j1);
        //TODO (Vitaliy Koshura): Check this!        
        const auto boneWidth = bodyPart.getBoneWidth(boneLength);

        auto maxSize = cv::Size(
          static_cast <uint32_t> (boneLength * resizeFactor),
          static_cast <uint32_t> (boneWidth * resizeFactor));
        if (result.size() > 0)
        {
          try
          {
            maxSize = result.at(bodyPart.getPartID());
          }
          catch (...) {}
        }
        result[bodyPart.getPartID()] = cv::Size(std::max(maxSize.width,
          static_cast <int> (boneLength * resizeFactor)), std::max(
            maxSize.height, static_cast <int> (boneWidth * resizeFactor)));
      }
    }
    // normalize
    for (auto &part : result)
    {
      part.second.width += (blockSize.width -
        part.second.width % blockSize.width);
      part.second.height += (blockSize.height -
        part.second.height % blockSize.height);
    }
    return result;
  }

  void HogDetector::train(const std::vector <Frame*> &frames,
    std::map <std::string, float> params)
  {
    m_partSize.clear();
    m_partModels.clear();

    emplaceDefaultParameters(params);

    m_bGrayImages = spelHelper::compareFloat(params.at(
      COMMON_HOG_DETECTOR_PARAMETERS::USE_GRAY_IMAGES().name()), 0.0f) == 0;

    auto bFirstConversion = true;
    Detector::train(frames, params, [&](auto frame, const auto scale) {
      if (bFirstConversion)
      {
        m_partSize = getMaxBodyPartHeightWidth(m_blockSize, scale);
        bFirstConversion = false;
      }

      m_partModels.insert(std::make_pair(frame->getID(),
        computeDescriptors(frame)));
    });
  }

  std::map <uint32_t, std::vector <LimbLabel> > HogDetector::detect(
    Frame *frame, std::map <std::string, float> params,
    const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const
  {
    auto detectorHelper = new HogDetectorHelper();

    emplaceDefaultParameters(params);

    auto result = Detector::detect(frame, params, limbLabels, detectorHelper);

    delete detectorHelper;
    frame->UnloadAll();
    return result;
  }

  LimbLabel HogDetector::generateLabel(const BodyPart &bodyPart, Frame *frame,
    const cv::Point2f &j0, const cv::Point2f &j1, DetectorHelper*,
    std::map <std::string, float> params) const
  {
    std::stringstream detectorName;
    detectorName << getID();

    emplaceDefaultParameters(params);

    const auto useHoGdet = params.at(
      COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR().name());

    cv::Size size;
    try
    {
      size = m_partSize.at(bodyPart.getPartID());
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get partSize for body part " << bodyPart.getPartID();
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }

    const auto &generatedPartModel = computeDescriptors(bodyPart, j0, j1,
      frame->getImage(), size);

    const auto &comparer = [&]()
    {
      return compare(bodyPart, generatedPartModel, m_nbins);
    };

    return Detector::generateLabel(bodyPart, j0, j1, detectorName.str(),
      useHoGdet, comparer);
  }

  float HogDetector::compare(const BodyPart &bodyPart, const PartModel &model,
    const uint8_t nbins) const
  {
    auto score = 0.0f;
    auto count = 0.0f;
    for (const auto &framePartModels : m_partModels)
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
        ss << "Invalid descriptor count. Need: " <<
          model.gradientStrengths.size() << ". Have: " <<
          partModel.gradientStrengths.size();
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
      for (auto i = 0U; i < model.gradientStrengths.size(); i++)
      {
        if (model.gradientStrengths.at(i).size() !=
          partModel.gradientStrengths.at(i).size())
        {
          std::stringstream ss;
          ss << "Invalid descriptor count. Need: " <<
            model.gradientStrengths.at(i).size() << ". Have: " <<
            partModel.gradientStrengths.at(i).size();
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
              score += std::abs(model.gradientStrengths.at(i).at(j).at(b) -
                partModel.gradientStrengths.at(i).at(j).at(b));
            }
            catch (...)
            {
              std::stringstream ss;
              ss << "Can't get some descriptor at [" << i << "][" << j <<
                "][" << b << "]";
              DebugMessage(ss.str(), 1);
              throw std::out_of_range(ss.str());
            }
          }
        }
      }
    }
    return (score / count);
  }

  std::map <uint32_t, std::map <uint32_t, HogDetector::PartModel>>
    HogDetector::getPartModels(void) const 
  {
    return m_partModels;
  }

  cv::Size HogDetector::getCellSize(void) const 
  {
    return m_cellSize;
  }

  uint8_t HogDetector::getnbins(void) const 
  {
    return m_nbins;
  }

  std::vector<std::vector<std::vector<float>>> HogDetector::calculateHog(
    const cv::Mat & image, std::vector<float>& descriptors,
    const cv::Size & wndSize, const cv::Size &blockSize,
    const cv::Size &blockStride, const cv::Size &cellSize, const int nbins,
    const int derivAperture, const double wndSigma,
    const int histogramNormType, const double thresholdL2hys,
    const bool gammaCorrection, const int nlevels)
  {
    descriptors.clear();
    cv::HOGDescriptor detector(wndSize, blockSize, blockStride, cellSize,
      nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys,
      gammaCorrection, nlevels);

    detector.compute(image, descriptors);

    std::vector <std::vector <uint32_t>> counter;

    std::vector <std::vector <std::vector <float>>> gradientStrengths;
    auto i = 0, j = 0;
    try
    {
      for (i = 0; i < wndSize.height; i += cellSize.height)
      {
        gradientStrengths.push_back(std::vector <std::vector <float>>());
        counter.push_back(std::vector <uint32_t>());
        for (j = 0; j < wndSize.width; j += cellSize.width)
        {
          gradientStrengths.at(i / cellSize.height).push_back(
            std::vector <float>());
          counter.at(i / cellSize.height).push_back(0);
          for (auto b = 0; b < nbins; b++)
            gradientStrengths.at(i / cellSize.height).at(j /
              cellSize.width).push_back(0.0f);
        }
      }
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get gradientStrengths at [" << i / cellSize.height <<
        "][" << j / cellSize.width << "]";
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }

    auto d = 0, n = 0, k = 0, r = 0, c = 0, b = 0;
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
                gradientStrengths.at(r / cellSize.height).at(c / cellSize.width).at(b) += descriptors.at(d);
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
      ss << "Descriptor parse error:" << std::endl << "Window row:\t" << n <<
        "\tWindow col:\t" << k << std::endl << "Block row:\t" << r <<
        "\tBlock col:\t" << c << std::endl << "NBins:\t" << b << std::endl;
      ss << "Total image rows:\t" << wndSize.height <<
        "\tTotal image cols:\t" << wndSize.width << std::endl;
      ss << "Total descriptors:\t" << descriptors.size() << std::endl;
      ss << "Trying to get descriptor at:\t" << d << std::endl;
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }

    try
    {
      for (i = 0; i < wndSize.height; i += cellSize.height)
      {
        for (j = 0; j < wndSize.width; j += cellSize.width)
        {
          for (b = 0; b < nbins; b++)
          {
            if (counter.at(i / cellSize.height).at(j / cellSize.width) ==
              0)
              gradientStrengths.at(i / cellSize.height).at(j /
                cellSize.width).at(b) = 0;
            else
              gradientStrengths.at(i / cellSize.height).at(j /
                cellSize.width).at(b) /= (counter.at(i /
                  cellSize.height).at(j / cellSize.width));
          }
        }
      }
    }
    catch (...)
    {
      std::stringstream ss;
      ss << "Can't get gradientStrengths at [" << i /
        cellSize.height << "][" << j / cellSize.width << "]";
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }

    return gradientStrengths;
  }

  void HogDetector::emplaceDefaultParameters(std::map<std::string, float>& params) const 
  {
    Detector::emplaceDefaultParameters(params);
    spelHelper::mergeParameters(params, COMMON_HOG_DETECTOR_PARAMETERS::getParameters());
  }

  HogDetectorHelper::HogDetectorHelper(void) 
  {
  }

  HogDetectorHelper::~HogDetectorHelper(void) 
  {
  }

}
