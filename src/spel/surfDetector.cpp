#include "surfDetector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "

namespace SPEL
{

  SurfDetector::SurfDetector(void)
  {
    id = 0x5344;
#if OpenCV_VERSION_MAJOR == 2 && OpenCV_VERSION_MINOR == 4 && OpenCV_VERSION_PATCH >= 9
    cv::initModule_nonfree();
#endif
  }

  SurfDetector::~SurfDetector(void)
  {
    for (auto i : partModels)
      for (auto j : i.second)
        j.second.descriptors.release();

    for (auto i : labelModels)
      for (auto j : i.second)
        for (auto k : j.second)
          k.descriptors.release();
  }

  int SurfDetector::getID(void) const
  {
    return id;
  }

  void SurfDetector::setID(int _id)
  {
    id = _id;
  }

  void SurfDetector::train(std::vector <Frame*> _frames, std::map <std::string, float> params)
  {
    frames = _frames;

    partModels.clear();
    labelModels.clear();

#ifdef DEBUG
    const uint8_t debugLevel = 5;
#else
    const uint8_t debugLevel = 1;
#endif // DEBUG
    const std::string sDebugLevel = "debugLevel";
    const uint32_t minHessian = 500;
    const std::string sMinHessian = "minHessian";

    params.emplace(sDebugLevel, debugLevel);
    params.emplace(sMinHessian, minHessian);

    const std::string sMaxFrameHeight = "maxFrameHeight";

    params.emplace(sMaxFrameHeight, frames.at(0)->getFrameSize().height);

    maxFrameHeight = params.at(sMaxFrameHeight);
    //maxFrameHeight=frames.at(0)->getFrameSize().height; //@TODO fix this later

    debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

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

      workFrame->Resize(params.at(sMaxFrameHeight));

      if (debugLevelParam >= 2)
        std::cerr << "Training on frame " << workFrame->getID() << std::endl;

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

  std::map <uint32_t, std::vector <LimbLabel> > SurfDetector::detect(Frame *frame, std::map <std::string, float> params, std::map <uint32_t, std::vector <LimbLabel>> limbLabels)
  {
    const std::string sMinHessian = "minHessian";
    const std::string sUseSURFdet = "useSURFdet";
    const std::string sKnnMatchCoeff = "knnMathCoeff";

    // first we need to check all used params
    params.emplace(sMinHessian, minHessian);
    params.emplace(sUseSURFdet, useSURFdet);
    params.emplace(sKnnMatchCoeff, knnMatchCoeff);

    //now set actual param values
    minHessian = params.at(sMinHessian);
    useSURFdet = params.at(sUseSURFdet);
    knnMatchCoeff = params.at(sKnnMatchCoeff);

    auto imgMat = frame->getImage();

#if OpenCV_VERSION_MAJOR == 3
#if defined (HAVE_OPENCV_XFEATURES2D)
    cv::Ptr <cv::xfeatures2d::SurfFeatureDetector> detector = cv::xfeatures2d::SurfFeatureDetector::create(minHessian);
#else
    cv::Ptr <cv::SurfFeatureDetector> detector = cv::SurfFeatureDetector::create(minHessian);
#endif // defined (HAVE_OPENCV_XFEATURES2D)
    detector->detect(imgMat, keyPoints);
#else
    cv::SurfFeatureDetector detector(minHessian);
    detector.detect(imgMat, keyPoints);
#endif // OpenCV_VERSION_MAJOR == 3
    if (keyPoints.empty())
    {
      std::stringstream ss;
      ss << ERROR_HEADER << "Couldn't detect keypoints for frame " << frame->getID();
      if (debugLevelParam >= 1)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::logic_error(ss.str());
    }

    auto result = Detector::detect(frame, params, limbLabels);

    keyPoints.clear();

    return result;
  }

  std::map <uint32_t, SurfDetector::PartModel> SurfDetector::computeDescriptors(Frame *frame, uint32_t minHessian)
  {
    std::map <uint32_t, PartModel> parts;
    Skeleton skeleton = frame->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    cv::Mat imgMat = frame->getImage();
    std::vector <cv::KeyPoint> keyPoints;

#if OpenCV_VERSION_MAJOR == 3
#if defined (HAVE_OPENCV_XFEATURES2D)
    cv::Ptr <cv::xfeatures2d::SurfFeatureDetector> detector = cv::xfeatures2d::SurfFeatureDetector::create(minHessian);
#else
    cv::Ptr <cv::SurfFeatureDetector> detector = cv::SurfFeatureDetector::create(minHessian);
#endif // defined (HAVE_OPENCV_XFEATURES2D)
    detector->detect(imgMat, keyPoints);
#else
    cv::SurfFeatureDetector detector(minHessian);
    detector.detect(imgMat, keyPoints);
#endif // OpenCV_VERSION_MAJOR == 3
    if (keyPoints.empty())
    {
      std::stringstream ss;
      ss << ERROR_HEADER << "Couldn't detect keypoints for frame " << frame->getID();
      if (debugLevelParam >= 1)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::logic_error(ss.str());
    }

    for (tree <BodyPart>::iterator part = partTree.begin(); part != partTree.end(); ++part)
    {
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
        parts.insert(std::pair <uint32_t, PartModel>(part->getPartID(), computeDescriptors(*part, j0, j1, imgMat, minHessian, keyPoints)));
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

  SurfDetector::PartModel SurfDetector::computeDescriptors(BodyPart bodyPart, cv::Point2f j0, cv::Point2f j1, cv::Mat imgMat, uint32_t minHessian, std::vector <cv::KeyPoint> keyPoints)
  {
    float boneLength = getBoneLength(j0, j1);
    float boneWidth = getBoneWidth(boneLength, bodyPart);
    cv::Size originalSize = cv::Size(static_cast <uint32_t> (boneLength), static_cast <uint32_t> (boneWidth));
    POSERECT <cv::Point2f> rect = getBodyPartRect(bodyPart, j0, j1, originalSize);

    PartModel partModel;
    partModel.partModelRect = rect;

    for (std::vector <cv::KeyPoint>::iterator kp = keyPoints.begin(); kp != keyPoints.end(); ++kp)
    {
      if (rect.containsPoint(kp->pt) > 0)
      {
        partModel.keyPoints.push_back(*kp);
      }
    }

    if (partModel.keyPoints.empty())
    {
      if (debugLevelParam >= 2)
        std::cerr << ERROR_HEADER << "Couldn't detect keypoints of body part " << bodyPart.getPartID() << std::endl;
    }
    else
    {
#if OpenCV_VERSION_MAJOR == 3
#if defined (HAVE_OPENCV_XFEATURES2D)
      cv::Ptr <cv::xfeatures2d::SurfDescriptorExtractor> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
#else
      cv::Ptr <cv::SurfDescriptorExtractor> extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();
#endif // defined (HAVE_OPENCV_XFEATURES2D)
      extractor->compute(imgMat, partModel.keyPoints, partModel.descriptors);
#else
      cv::SurfDescriptorExtractor extractor;
      extractor.compute(imgMat, partModel.keyPoints, partModel.descriptors);
#endif // OpenCV_VERSION_MAJOR == 3
      if (partModel.descriptors.empty() && debugLevelParam >= 2)
      {
        std::cerr << ERROR_HEADER << "Couldn't compute descriptors of body part " << bodyPart.getPartID() << std::endl;
      }
    }
    return partModel;
  }

  LimbLabel SurfDetector::generateLabel(BodyPart bodyPart, Frame *frame, cv::Point2f j0, cv::Point2f j1)
  {
    std::stringstream detectorName;
    detectorName << getID();

    comparer_bodyPart = &bodyPart;

    PartModel generatedPartModel = computeDescriptors(bodyPart, j0, j1, frame->getImage(), minHessian, keyPoints);

    comparer_model = &generatedPartModel;
    comparer_j0 = &j0;
    comparer_j1 = &j1;

    LimbLabel label = Detector::generateLabel(bodyPart, j0, j1, detectorName.str(), useSURFdet);

    labelModels[frame->getID()][bodyPart.getPartID()].push_back(generatedPartModel);

    comparer_bodyPart = 0;
    comparer_model->descriptors.release();
    comparer_model = 0;
    comparer_j0 = 0;
    comparer_j1 = 0;

    return label;
  }

  float SurfDetector::compare(void)
  {
    if (comparer_bodyPart == 0 || comparer_model == 0 || comparer_j0 == 0 || comparer_j1 == 0)
    {
      std::stringstream ss;
      ss << "Compare parameters are invalid: " << (comparer_bodyPart == 0 ? "comparer_bodyPart == 0 " : "") << (comparer_model == 0 ? "comparer_model == 0 " : "") << (comparer_j0 == 0 ? "comparer_j0 == 0 " : "") << (comparer_j1 == 0 ? "comparer_j1 == 0" : "") << std::endl;
      if (debugLevelParam >= 1)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::logic_error(ss.str());
    }
    return compare(*comparer_bodyPart, *comparer_model, *comparer_j0, *comparer_j1);
  }

  float SurfDetector::compare(BodyPart bodyPart, PartModel model, cv::Point2f j0, cv::Point2f j1)
  {
    if (model.descriptors.empty())
    {
      if (debugLevelParam >= 2)
        std::cerr << ERROR_HEADER << "Model descriptors are empty" << std::endl;
      return -1.0f;
    }

    float score = 0;
    uint32_t count = 0;
    cv::FlannBasedMatcher matcher;
    std::vector <std::vector <cv::DMatch>> matches;

    float length = getBoneLength(j0, j1);
    float width = getBoneWidth(length, bodyPart);
    float coeff = sqrt(pow(length, 2) + pow(width, 2));

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
          if (partModel->second.descriptors.empty() || model.descriptors.empty())
          {
            if (debugLevelParam >= 2)
              std::cerr << ERROR_HEADER << "PartModel descriptors of body part [" << partModel->first << "] are empty" << std::endl;
          }
          else
          {
            try
            {
              if (partModel->second.descriptors.rows > 1 && model.descriptors.rows > 1)
              {
                matcher.knnMatch(model.descriptors, partModel->second.descriptors, matches, 2);
                float s = 0;
                for (uint32_t i = 0; i < matches.size(); i++)
                {
                  if ((matches[i][0].distance < knnMatchCoeff * (matches[i][1].distance)) && ((int)matches[i].size() <= 2 && (int)matches[i].size()>0))
                  {
                    s += matches[i][0].distance / coeff;
                    count++;
                  }
                }
                score += s / matches.size();
              }
              else
              {
                if (debugLevelParam >= 1)
                  std::cerr << ERROR_HEADER << "Can't match descriptors of body part [" << partModel->first << "]: Not enough descriptors" << std::endl;
              }
            }
            catch (...)
            {
              if (debugLevelParam >= 1)
                std::cerr << ERROR_HEADER << "Can't match descriptors of body part [" << partModel->first << "]" << std::endl;
            }
          }
          break;
        }
      }
    }
    if (count == 0)
      return -1.0f;
    return (score / (float)count);
  }

  std::map <uint32_t, std::map <uint32_t, SurfDetector::PartModel>> SurfDetector::getPartModels(void)
  {
    return partModels;
  }

  std::map <uint32_t, std::map <uint32_t, std::vector <SurfDetector::PartModel>>> SurfDetector::getLabelModels(void)
  {
    return labelModels;
  }

}
