#ifndef _LIBPOSE_SURFDETECTOR_HPP_
#define _LIBPOSE_SURFDETECTOR_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <utility>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#if OpenCV_VERSION_MAJOR == 3 && defined (HAVE_OPENCV_XFEATURES2D)
#include "opencv2/features2d.hpp"
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#elif defined (HAVE_OPENCV_FEATURES2D)
#if OpenCV_VERSION_MAJOR == 2 && OpenCV_VERSION_MINOR == 4 && OpenCV_VERSION_PATCH >= 9
#include <opencv2/nonfree/nonfree.hpp>
#else
#warning "Unsupported version of OpenCV"
#include <opencv2/features2d/features2d.hpp>
#endif
#else
#error "Unsupported version of OpenCV"
#endif

#include "detector.hpp"

namespace SPEL
{
  class SurfDetectorHelper : public DetectorHelper
  {
  public:
    SurfDetectorHelper(void) noexcept;
    virtual ~SurfDetectorHelper(void) noexcept;
    std::vector <cv::KeyPoint> keyPoints;
  };

  class SurfDetector : public Detector
  {
  protected:
    struct PartModel
    {
      virtual ~PartModel(void);
      POSERECT <cv::Point2f> partModelRect;
      std::vector <cv::KeyPoint> keyPoints;
      cv::Mat descriptors;
    };
  public:
    SurfDetector(void) noexcept;
    virtual ~SurfDetector(void) noexcept;
    virtual void train(const std::vector <Frame*> &_frames, std::map <std::string, float>);
    virtual std::map <uint32_t, std::vector <LimbLabel> > detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const;
    virtual std::map <uint32_t, std::map <uint32_t, PartModel>> getPartModels(void) const noexcept;

  private:
#ifdef DEBUG
    FRIEND_TEST(surfDetectorTests, computeDescriptors);
    FRIEND_TEST(surfDetectorTests, train);
    FRIEND_TEST(surfDetectorTests, compare);
    FRIEND_TEST(surfDetectorTests, generateLabel);
    FRIEND_TEST(surfDetectorTests, PartModel);
    FRIEND_TEST(surfDetectorTests, getPartModels);
    //FRIEND_TEST(surfDetectorTests, detect);
#endif  // DEBUG
  protected:
    std::map <uint32_t, std::map <uint32_t, PartModel>> partModels;

    virtual std::map <uint32_t, PartModel> computeDescriptors(const Frame *frame, const uint32_t minHessian) const;
    virtual PartModel computeDescriptors(const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1, const cv::Mat &imgMat, const uint32_t minHessian, const std::vector <cv::KeyPoint> &keyPoints) const;
    virtual LimbLabel generateLabel(const BodyPart &bodyPart, const Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1, DetectorHelper *detectorHelper, std::map <std::string, float> params) const;
    virtual float compare(const BodyPart &bodyPart, const PartModel &model, const cv::Point2f &j0, const cv::Point2f &j1, const float knnMatchCoeff) const;
  };
}

#endif  // _LIBPOSE_SURFDETECTOR_HPP_ctujly
