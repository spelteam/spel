// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
#include "surfDetector.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"
#include "spelParameters.hpp"

namespace SPEL
{
  SurfDetector::SurfDetector(void) 
  {
    m_id = 0x53440000;
  }

  SurfDetector::~SurfDetector(void) 
  {
    for (auto &i : partModels)
      for (auto &j : i.second)
        j.second.descriptors.release();

  }

  void SurfDetector::train(const std::vector <Frame*> &frames,
    std::map <std::string, float> params)
  {
    partModels.clear();

    emplaceDefaultParameters(params);

    Detector::train(frames, params, [&](auto frame, const auto) {
      const auto minHessian = static_cast<uint32_t> (params.at(
        COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN().name()));

      partModels.insert(std::make_pair(frame->getID(),
        computeDescriptors(frame, minHessian)));
    });
  }

  std::map <uint32_t, std::vector <LimbLabel> > SurfDetector::detect(
    Frame *frame, std::map <std::string, float> params, 
    const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const
  {
    emplaceDefaultParameters(params);

    //now set actual param values
    const auto minHessian = params.at(
      COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN().name());
    auto detectorHelper = new SurfDetectorHelper();
    const auto &imgMat = frame->getImage();
    // - all coordinates of key points must have identical scale 
    // with the skeleton

    auto detector = cv::xfeatures2d::SurfFeatureDetector::create(minHessian);
    detector->detect(imgMat, detectorHelper->keyPoints);
    if (detectorHelper->keyPoints.empty())
    {
      frame->UnloadAll();
      delete detectorHelper;
      std::stringstream ss;
      ss << "Couldn't detect keypoints for frame " << frame->getID();
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }

    auto result = Detector::detect(frame, params, limbLabels, detectorHelper);

    delete detectorHelper;
    frame->UnloadAll();
    return result;
  }

  std::map <uint32_t, SurfDetector::PartModel>
    SurfDetector::computeDescriptors(Frame *frame, 
      const uint32_t minHessian) const
  {
    std::map <uint32_t, PartModel> parts;
    const auto &skeleton = frame->getSkeleton();
    const auto &partTree = skeleton.getPartTree();
    const auto &imgMat = frame->getImage();
    std::vector <cv::KeyPoint> keyPoints;

    auto detector = cv::xfeatures2d::SurfFeatureDetector::create(minHessian);
    detector->detect(imgMat, keyPoints);
    if (keyPoints.empty())
    {
      std::stringstream ss;
      ss << "Couldn't detect keypoints for frame " << frame->getID();
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }

    for (const auto &part : partTree)
    {
      auto joint = skeleton.getBodyJoint(part.getParentJoint());
      if (joint == 0)
      {
        const auto &str = "Invalid parent joint";
        DebugMessage(str, 1);
        throw std::logic_error(str);
      }
      const auto &j0 = joint->getImageLocation();
      joint = 0;
      joint = skeleton.getBodyJoint(part.getChildJoint());
      if (joint == 0)
      {
        const auto &str = "Invalid child joint";
        DebugMessage(str, 1);
        throw std::logic_error(str);
      }
      const auto &j1 = joint->getImageLocation();
      try
      {
        parts.insert(std::pair <uint32_t, PartModel>(part.getPartID(), 
          computeDescriptors(part, j0, j1, imgMat, keyPoints)));
      }
      catch (std::logic_error &err)
      {
        std::stringstream ss;
        ss << "Can't compute descriptors for the frame " << frame->getID() << 
          " for the part " << part.getPartID() << std::endl;
        ss << "\t" << err.what();
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
    }
    frame->UnloadAll();
    return parts;
  }

  SurfDetector::PartModel SurfDetector::computeDescriptors(
    const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1, 
    const cv::Mat &imgMat, const std::vector <cv::KeyPoint> &keyPoints) const
  {
    const auto boneLength = BodyPart::getBoneLength(j0, j1);
    const auto boneWidth = bodyPart.getBoneWidth(boneLength);
    const auto &originalSize = cv::Size(static_cast <uint32_t> (boneLength), 
      static_cast <uint32_t> (boneWidth));
    const auto &rect = 
      BodyPart::getBodyPartRect(bodyPart, j0, j1, originalSize);

    PartModel partModel;
    partModel.partModelRect = rect;

    for (const auto &kp : keyPoints)
      if (rect.containsPoint(kp.pt) > 0)
        partModel.keyPoints.push_back(kp);

    if (partModel.keyPoints.empty())
    {
      if (SpelObject::getDebugLevel() >= 2)
      {
        std::stringstream ss;
        ss << "Couldn't detect keypoints of body part " << bodyPart.getPartID();
        DebugMessage(ss.str(), 2);
      }
    }
    else
    {
      auto extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
      extractor->compute(imgMat, partModel.keyPoints, partModel.descriptors);
      if (partModel.descriptors.empty())
      {
        if (SpelObject::getDebugLevel() >= 2)
        {
          std::stringstream ss;
          ss << "Couldn't compute descriptors of body part " <<
            bodyPart.getPartID();
          DebugMessage(ss.str(), 2);
        }
      }
    }
    return partModel;
  }

  LimbLabel SurfDetector::generateLabel(const BodyPart &bodyPart,
    Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1, 
    DetectorHelper *detectorHelper, std::map <std::string, float> params) const
  {
    std::stringstream detectorName;
    detectorName << getID();

    SurfDetectorHelper* helper = 0;
    try
    {
      helper = dynamic_cast<SurfDetectorHelper*> (detectorHelper);
    }
    catch (...)
    {
      const auto &ss = "Wrong type: detectorHelper is not SurfDetectorHelper";
      throw std::logic_error(ss);
    }

    emplaceDefaultParameters(params);

    const auto useSURFdet = params.at(
      COMMON_DETECTOR_PARAMETERS::USE_SURF_DETECTOR().name());
    const auto knnMatchCoeff = params.at(
      COMMON_SURF_DETECTOR_PARAMETERS::KNN_MATCH_COEFFICIENT().name());

    const auto &generatedPartModel = computeDescriptors(bodyPart, j0, j1, 
      frame->getImage(), helper->keyPoints);

    const auto &comparer = [&]()
    {
      return compare(bodyPart, generatedPartModel, j0, j1, knnMatchCoeff);
    };

    return Detector::generateLabel(bodyPart, j0, j1, detectorName.str(), 
      useSURFdet, comparer);
  }

  float SurfDetector::compare(const BodyPart &bodyPart, const PartModel &model,
    const cv::Point2f &j0, const cv::Point2f &j1, const float knnMatchCoeff) 
    const
  {
    if (model.descriptors.empty())
    {
      if (SpelObject::getDebugLevel() >= 2)
      {
        const auto &str = "Model descriptors are empty";
        DebugMessage(str, 2);
      }
      return -1.0f;
    }

    auto score = 0.0f;
    auto count = 0U;
    cv::FlannBasedMatcher matcher;
    std::vector <std::vector <cv::DMatch>> matches;

    const auto length = BodyPart::getBoneLength(j0, j1);
    const auto width = bodyPart.getBoneWidth(length);
    const auto coeff = sqrt(pow(length, 2) + pow(width, 2));

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

      if (partModel.descriptors.empty())
      {
        if (SpelObject::getDebugLevel() >= 2)
        {
          std::stringstream ss;
          ss << "PartModel descriptors of body part [" <<
            bodyPart.getPartID() << "] are empty";
          DebugMessage(ss.str(), 2);
        }
      }
      else
      {
        try
        {
          if (partModel.descriptors.rows > 1 && model.descriptors.rows > 1)
          {
            matcher.knnMatch(model.descriptors, partModel.descriptors, 
              matches, 2);
            auto s = 0.0f;
            for (const auto &i : matches)
            {
              if (i.size() == 2 && (i[0].distance < knnMatchCoeff * 
                (i[1].distance)))
              {
                s += i[0].distance / coeff;
                count++;
              }
            }
            score += s / matches.size();
          }
          else
          {
            if (SpelObject::getDebugLevel() >= 2)
            {
              std::stringstream ss;
              ss << "Can't match descriptors of body part [" <<
                bodyPart.getPartID() << "]: Not enough descriptors";
              DebugMessage(ss.str(), 2);
            }
          }
        }
        catch (...)
        {
          if (SpelObject::getDebugLevel() >= 2)
          {
            std::stringstream ss;
            ss << "Can't match descriptors of body part [" <<
              bodyPart.getPartID() << "]";
            DebugMessage(ss.str(), 2);
          }
        }
      }
    }
    if (count == 0)
      return -1.0f;
    return (score / static_cast<float>(count));
  }

  std::map <uint32_t, std::map <uint32_t, SurfDetector::PartModel>>
    SurfDetector::getPartModels(void) const 
  {
    return partModels;
  }

  void SurfDetector::emplaceDefaultParameters(std::map<std::string, float>& params) const 
  {
    Detector::emplaceDefaultParameters(params);
    spelHelper::mergeParameters(params, COMMON_SURF_DETECTOR_PARAMETERS::getParameters());
  }

  SurfDetectorHelper::SurfDetectorHelper(void) 
  {
  }

  SurfDetectorHelper::~SurfDetectorHelper(void) 
  {
  }

  SurfDetector::PartModel::~PartModel(void) 
  {
    descriptors.release();
  }
}
