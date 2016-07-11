#include "surfDetector.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"
#include "spelParameters.hpp"

namespace SPEL
{

  SurfDetector::SurfDetector(void) noexcept
  {
    m_id = 0x53440000;
#if OpenCV_VERSION_MAJOR == 2 && OpenCV_VERSION_MINOR == 4 && OpenCV_VERSION_PATCH >= 9
    cv::initModule_nonfree();
#endif
  }

  SurfDetector::~SurfDetector(void) noexcept
  {
    for (auto &i : partModels)
      for (auto &j : i.second)
        j.second.descriptors.release();

  }

  void SurfDetector::train(const std::vector <Frame*> &_frames, std::map <std::string, float> params)
  {
    frames = _frames;

    partModels.clear();

    params.emplace(COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN());
    params.emplace(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().first,
      frames.at(0)->getFrameSize().height);

    auto minHessian = static_cast<uint32_t> (params.at(COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN().first));
    maxFrameHeight = params.at(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().first);

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

      workFrame = frameNum->clone(workFrame);

      workFrame->Resize(maxFrameHeight);

      if (SpelObject::getDebugLevel() >= 2)
        std::cout << "Training on frame " << workFrame->getID() << std::endl;

      try
      {
        partModels.insert(std::pair <uint32_t, std::map <uint32_t, PartModel>>(workFrame->getID(), computeDescriptors(workFrame, minHessian)));
      }
      catch (...)
      {
        break;
      }

      delete workFrame;
    }
  }

  std::map <uint32_t, std::vector <LimbLabel> > SurfDetector::detect(Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const
  {
    // first we need to check all used params
    params.emplace(COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN());
    params.emplace(COMMON_DETECTOR_PARAMETERS::USE_SURF_DETECTOR());
    params.emplace(COMMON_SURF_DETECTOR_PARAMETERS::KNN_MATCH_COEFFICIENT());

    //now set actual param values
    auto minHessian = params.at(COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN().first);
    auto imgMat = frame->getImage();
    auto detectorHelper = new SurfDetectorHelper();

    // Resize frame - added 02.07.16 (is a "ñrutch")
    Frame *workFrame = nullptr;
    if (frame->getFrametype() == KEYFRAME)
      workFrame = new Keyframe();
    else if (frame->getFrametype() == LOCKFRAME)
      workFrame = new Lockframe();
    else if (frame->getFrametype() == INTERPOLATIONFRAME)
      workFrame = new Interpolation();
    workFrame = frame->clone(workFrame);
    float resizeFactor = workFrame->Resize(maxFrameHeight);
    imgMat = workFrame->getImage();
    // - all coordinates of key points must have identical scale with the skeleton

#if OpenCV_VERSION_MAJOR == 3
#if defined (HAVE_OPENCV_XFEATURES2D)
    auto detector = cv::xfeatures2d::SurfFeatureDetector::create(minHessian);
#else
    auto detector = cv::SurfFeatureDetector::create(minHessian);
#endif // defined (HAVE_OPENCV_XFEATURES2D)
    detector->detect(imgMat, detectorHelper->keyPoints);
#else
    cv::SurfFeatureDetector detector(minHessian);
    detector.detect(imgMat, detectorHelper->keyPoints);
#endif // OpenCV_VERSION_MAJOR == 3
    if (detectorHelper->keyPoints.empty())
    {
      workFrame->UnloadAll(); // added 02.07.16		
      frame->UnloadAll();
      delete detectorHelper;
      std::stringstream ss;
      ss << "Couldn't detect keypoints for frame " << frame->getID();
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }

    auto result = Detector::detect(workFrame, params, limbLabels, detectorHelper); // 02.07.16 changed "frame" to "workFrame" (this is a "ñrutch")

    workFrame->UnloadAll(); // added 02.07.16
    delete detectorHelper;
    frame->UnloadAll();


    return result;
  }

  std::map <uint32_t, SurfDetector::PartModel> SurfDetector::computeDescriptors(Frame *frame, const uint32_t minHessian) const
  {
    std::map <uint32_t, PartModel> parts;
    auto skeleton = frame->getSkeleton();
    auto partTree = skeleton.getPartTree();
    auto imgMat = frame->getImage();
    std::vector <cv::KeyPoint> keyPoints;

#if OpenCV_VERSION_MAJOR == 3
#if defined (HAVE_OPENCV_XFEATURES2D)
    auto detector = cv::xfeatures2d::SurfFeatureDetector::create(minHessian);
#else
    auto detector = cv::SurfFeatureDetector::create(minHessian);
#endif // defined (HAVE_OPENCV_XFEATURES2D)
    detector->detect(imgMat, keyPoints);
#else
    cv::SurfFeatureDetector detector(minHessian);
    detector.detect(imgMat, keyPoints);
#endif // OpenCV_VERSION_MAJOR == 3
    if (keyPoints.empty())
    {
      frame->UnloadAll();
      std::stringstream ss;
      ss << "Couldn't detect keypoints for frame " << frame->getID();
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }

    for (const auto &part : partTree)
    {
      auto *joint = skeleton.getBodyJoint(part.getParentJoint());
      if (joint == 0)
      {
        frame->UnloadAll();
        const std::string str = "Invalid parent joint";
        DebugMessage(str, 1);
        throw std::logic_error(str);
      }
      auto j0 = joint->getImageLocation();
      joint = 0;
      joint = skeleton.getBodyJoint(part.getChildJoint());
      if (joint == 0)
      {
        frame->UnloadAll();
        const std::string str = "Invalid child joint";
        DebugMessage(str, 1);
        throw std::logic_error(str);
      }
      auto j1 = joint->getImageLocation();
      try
      {
        parts.insert(std::pair <uint32_t, PartModel>(part.getPartID(), computeDescriptors(part, j0, j1, imgMat, minHessian, keyPoints)));
      }
      catch (std::logic_error err)
      {
        frame->UnloadAll();
        std::stringstream ss;
        ss << "Can't compute descriptors for the frame " << frame->getID() << " for the part " << part.getPartID() << std::endl;
        ss << "\t" << err.what();
        DebugMessage(ss.str(), 1);
        throw std::logic_error(ss.str());
      }
    }
    frame->UnloadAll();
    return parts;
  }

  SurfDetector::PartModel SurfDetector::computeDescriptors(const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1, const cv::Mat &imgMat, const uint32_t minHessian, const std::vector <cv::KeyPoint> &keyPoints) const
  {
    auto boneLength = BodyPart::getBoneLength(j0, j1);
    auto boneWidth = bodyPart.getBoneWidth(boneLength);
    auto originalSize = cv::Size(static_cast <uint32_t> (boneLength), static_cast <uint32_t> (boneWidth));
    auto rect = BodyPart::getBodyPartRect(bodyPart, j0, j1, originalSize);

    PartModel partModel;
    partModel.partModelRect = rect;

    for (const auto &kp : keyPoints)
      if (rect.containsPoint(kp.pt) > 0)
        partModel.keyPoints.push_back(kp);

    if (partModel.keyPoints.empty())
    {
      std::stringstream ss;
      ss << "Couldn't detect keypoints of body part " << bodyPart.getPartID();
      DebugMessage(ss.str(), 2);
    }
    else
    {
#if OpenCV_VERSION_MAJOR == 3
#if defined (HAVE_OPENCV_XFEATURES2D)
      auto extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
#else
      auto extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
#endif // defined (HAVE_OPENCV_XFEATURES2D)
      extractor->compute(imgMat, partModel.keyPoints, partModel.descriptors);
#else
      cv::SurfDescriptorExtractor extractor;
      extractor.compute(imgMat, partModel.keyPoints, partModel.descriptors);
#endif // OpenCV_VERSION_MAJOR == 3
      if (partModel.descriptors.empty())
      {
        std::stringstream ss;
        ss << "Couldn't compute descriptors of body part " << bodyPart.getPartID();
        DebugMessage(ss.str(), 2);
      }
    }
    return partModel;
  }

  LimbLabel SurfDetector::generateLabel(const BodyPart &bodyPart, Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1, DetectorHelper *detectorHelper, std::map <std::string, float> params) const
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
      std::stringstream ss;
      ss << "Wrong type: detectorHelper is not SurfDetectorHelper";
      throw std::logic_error(ss.str());
    }

    params.emplace(COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN());
    params.emplace(COMMON_DETECTOR_PARAMETERS::USE_SURF_DETECTOR());
    params.emplace(COMMON_SURF_DETECTOR_PARAMETERS::KNN_MATCH_COEFFICIENT());

    auto minHessian = static_cast<uint32_t> (params.at(COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN().first));
    auto useSURFdet = params.at(COMMON_DETECTOR_PARAMETERS::USE_SURF_DETECTOR().first);
    auto knnMatchCoeff = params.at(COMMON_SURF_DETECTOR_PARAMETERS::KNN_MATCH_COEFFICIENT().first);

    auto generatedPartModel = computeDescriptors(bodyPart, j0, j1, frame->getImage(), minHessian, helper->keyPoints);

    auto comparer = [&]() -> float
    {
      return compare(bodyPart, generatedPartModel, j0, j1, knnMatchCoeff);
    };

    auto label = Detector::generateLabel(bodyPart, j0, j1, detectorName.str(), useSURFdet, comparer);
    frame->UnloadAll();
    return label;
  }

  float SurfDetector::compare(const BodyPart &bodyPart, const PartModel &model, const cv::Point2f &j0, const cv::Point2f &j1, const float knnMatchCoeff) const
  {
    if (model.descriptors.empty())
    {
      const std::string str = "Model descriptors are empty";
      DebugMessage(str, 2);
      return -1.0f;
    }

    auto score = 0.0f;
    auto count = 0U;
    cv::FlannBasedMatcher matcher;
    std::vector <std::vector <cv::DMatch>> matches;

    auto length = BodyPart::getBoneLength(j0, j1);
    auto width = bodyPart.getBoneWidth(length);
    auto coeff = sqrt(pow(length, 2) + pow(width, 2));

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

      if (partModel.descriptors.empty() || model.descriptors.empty())
      {
        std::stringstream ss;       
        ss << "PartModel descriptors of body part [" << bodyPart.getPartID() << "] are empty";
        DebugMessage(ss.str(), 2);
      }
      else
      {
        try
        {
          if (partModel.descriptors.rows > 1 && model.descriptors.rows > 1)
          {
            matcher.knnMatch(model.descriptors, partModel.descriptors, matches, 2);
            auto s = 0.0f;
            for (const auto &i : matches)
            {
              if (i.size() == 2 && (i[0].distance < knnMatchCoeff * (i[1].distance)))
              {
                s += i[0].distance / coeff;
                count++;
              }
            }
            score += s / matches.size();
          }
          else
          {
            std::stringstream ss;
            ss << "Can't match descriptors of body part [" << bodyPart.getPartID() << "]: Not enough descriptors";
            DebugMessage(ss.str(), 1);
          }
        }
        catch (...)
        {
          std::stringstream ss;
          ss << "Can't match descriptors of body part [" << bodyPart.getPartID() << "]";
          DebugMessage(ss.str(), 1);
        }
      }
    }
    if (count == 0)
      return -1.0f;
    return (score / static_cast<float>(count));
  }

  std::map <uint32_t, std::map <uint32_t, SurfDetector::PartModel>> SurfDetector::getPartModels(void) const noexcept
  {
    return partModels;
  }

  SurfDetectorHelper::SurfDetectorHelper(void) noexcept
  {
  }

  SurfDetectorHelper::~SurfDetectorHelper(void) noexcept
  {
  }

  SurfDetector::PartModel::~PartModel(void) noexcept
  {
    descriptors.release();
  }
}
