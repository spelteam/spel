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
    int histogramNormType) :
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

  HogDetector::PartModel HogDetector::computeDescriptors(const cv::Mat &imgMat,
    const spelRECT <cv::Point2f> &rect, const cv::Size &wndSize) 
    const
  {
    const auto &size = rect.RectSize<cv::Size2f>();

    auto boneLength = static_cast<int>(size.height);
    if (boneLength < m_blockSize.width)
      boneLength = m_blockSize.width;
    else
      boneLength = boneLength + m_blockSize.width - (boneLength % 
        m_blockSize.width);

    auto boneWidth = static_cast<int>(size.width);
    if (boneWidth < m_blockSize.height)
      boneWidth = m_blockSize.height;
    else
      boneWidth = boneWidth + m_blockSize.width - (boneWidth % 
        m_blockSize.height);

    const auto &originalSize = cv::Size(static_cast <uint32_t> (boneLength),
      static_cast <uint32_t> (boneWidth));

    const auto &direction = rect.point3 - rect.point2;
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

  HogDetector::PartModel HogDetector::computeDescriptors (
    const cv::Mat &imgMat, const cv::Size &wndSize, LimbLabel &label) const
  {
    return computeDescriptors(imgMat, label.getRect(), wndSize);
  }

  HogDetector::PartModel HogDetector::computeDescriptors(
    const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1,
    const cv::Mat &imgMat, const cv::Size &wndSize) const
  {
    const auto &rect = bodyPart.getBodyPartRect(j0, j1, m_blockSize);
    return computeDescriptors(imgMat, rect, wndSize);
  }

  std::map <uint32_t, HogDetector::PartModel> HogDetector::computeDescriptors(
    Frame *frame) const
  {
    std::map <uint32_t, PartModel> parts;
    const auto &skeleton = frame->getSkeleton();
    const auto &partTree = skeleton.getPartTree();
    for (const auto &part : partTree)
    {
      const auto &wndSize = m_partSize.at(part.getPartID());
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
          const auto r = result.find(bodyPart.getPartID());
          if (r != result.end())
            maxSize = r->second;
        }
        const auto &bodyPartSize = cv::Size(std::max(maxSize.width,
          static_cast <int> (boneLength * resizeFactor)), std::max(
            maxSize.height, static_cast <int> (boneWidth * resizeFactor)));
        result.insert(std::make_pair(bodyPart.getPartID(), bodyPartSize));
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
        m_partSize = this->getMaxBodyPartHeightWidth(m_blockSize, scale);
        bFirstConversion = false;
      }

      m_partModels.insert(std::make_pair(frame->getID(),
        this->computeDescriptors(frame)));
    });
  }

  std::map <uint32_t, std::vector <LimbLabel> > HogDetector::detect(
    Frame *frame, std::map <std::string, float> params,
    std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const
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

    const auto &size = m_partSize.at(bodyPart.getPartID());

    const auto &generatedPartModel = computeDescriptors(bodyPart, j0, j1,
      frame->getImage(), size);

    const auto &comparer = [&]()
    {
      return compare(bodyPart, generatedPartModel, m_nbins);
    };

    return Detector::generateLabel(bodyPart, j0, j1, detectorName.str(),
      useHoGdet, comparer);
  }

  void HogDetector::calculateLabelScore(Frame * workFrame, DetectorHelper*, LimbLabel & label, std::map<std::string, float> params) const
  {
    std::stringstream detectorName;
    detectorName << getID();

    emplaceDefaultParameters(params);

    const auto useHoGdet = params.at(
      COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR().name());

    const auto &size = m_partSize.at(label.getLimbID());

    const auto &generatedPartModel = computeDescriptors(workFrame->getImage(), 
      size, label);

    const auto &comparer = [&]()
    {
      return compare(label, generatedPartModel, m_nbins);
    };

    Detector::addLabelScore(label, detectorName.str(), useHoGdet, comparer);
  }

  float HogDetector::compare(const BodyPart &bodyPart, const PartModel &model,
    const uint8_t nbins) const
  {
    return compare(model, bodyPart.getPartID(), nbins);
  }

  float HogDetector::compare(const LimbLabel & label, const PartModel & partModel, const uint8_t nbins) const
  {
    return compare(partModel, label.getLimbID(), nbins);
  }

  float HogDetector::compare(const PartModel & model, const int partId, const uint8_t nbins) const
  {
    auto score = 0.0f;
    auto count = 0.0f;
    for (const auto &framePartModels : m_partModels)
    {
      const auto pm = framePartModels.second.find(partId);
      if (pm == framePartModels.second.end())
      {
        std::stringstream ss;
        ss << "Can't find part model for body part " << partId;
        DebugMessage(ss.str(), 1);
        throw std::out_of_range(ss.str());
      }
      const auto &partModel = pm->second;
      if (model.gradientStrengths.size() != partModel.gradientStrengths.size())
      {
        std::stringstream ss;
        ss << "Invalid descriptor count. Need: " <<
          model.gradientStrengths.size() << ". Have: " <<
          partModel.gradientStrengths.size();
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
      for (decltype(model.gradientStrengths.size()) i = 0U; i < model.gradientStrengths.size(); ++i)
      {
        const auto &mi = model.gradientStrengths[i];
        const auto &pi = partModel.gradientStrengths[i];
        if (mi.size() != pi.size())
        {
          std::stringstream ss;
          ss << "Invalid descriptor count. Need: " <<
            mi.size() << ". Have: " << pi.size();
          DebugMessage(ss.str(), 1);
          throw std::logic_error(ss.str());
        }
        for (decltype(mi.size()) j = 0U; j < mi.size(); ++j)
        {
          const auto &mj = mi[j];
          const auto &pj = pi[j];
          for (auto b = 0; b < nbins; ++b)
          {
            ++count;
            score += std::abs(mj[b] - pj[b]);
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

    std::vector <std::vector <uint32_t>> counter(
      wndSize.height / cellSize.height, std::vector <uint32_t>(
        wndSize.width / cellSize.width, 0));

    std::vector <std::vector <std::vector <float>>> gradientStrengths(
      wndSize.height / cellSize.height, std::vector <std::vector<float>>(
        wndSize.width / cellSize.width, std::vector<float>(nbins, 0.0f)));

    auto d = 0U;
    // window rows
    for (auto n = 0; n + blockStride.height < wndSize.height; n += blockStride.height)
    {
      // window cols
      for (auto k = 0; k + blockStride.width < wndSize.width; k += blockStride.width)
      {
        // block rows
        for (auto r = n; r < n + blockSize.height; r += cellSize.height)
        {
          // block cols
          for (auto c = k; c < k + blockSize.width; c += cellSize.width)
          {
            // nbins
            for (auto b = 0; b < nbins; ++b)
            {
              gradientStrengths[r / cellSize.height][c / cellSize.width][b] += descriptors[d];
              ++d;
            }
            ++(counter[r / cellSize.height][c / cellSize.width]);
          }
        }
      }
    }

    for (auto i = 0; i < wndSize.height; i += cellSize.height)
    {
      for (auto j = 0; j < wndSize.width; j += cellSize.width)
      {
        for (auto b = 0; b < nbins; ++b)
        {
          if (counter[i / cellSize.height][j / cellSize.width] == 0)
            gradientStrengths[i / cellSize.height][j / cellSize.width][b] = 0;
          else
            gradientStrengths[i / cellSize.height][j / cellSize.width][b] /=
            (counter[i / cellSize.height][j / cellSize.width]);
        }
      }
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
