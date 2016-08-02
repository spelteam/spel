#ifndef _LIBPOSE_SURFDETECTOR_HPP_
#define _LIBPOSE_SURFDETECTOR_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <utility>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#if !defined (HAVE_OPENCV_XFEATURES2D)
#error "Unsupported version of OpenCV. XFeatures2D are not installed."
#endif
#include "opencv2/features2d.hpp"
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d/features2d.hpp>

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
    void train(const std::vector <Frame*> &_frames, 
      std::map <std::string, float>);
    std::map <uint32_t, std::vector <LimbLabel> > detect(Frame *frame, 
      std::map <std::string, float> params, 
      const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const;
    std::map <uint32_t, std::map <uint32_t, PartModel>> getPartModels(void) 
      const noexcept;

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

    std::map <uint32_t, PartModel> computeDescriptors(Frame *frame, 
      const uint32_t minHessian) const;
    PartModel computeDescriptors(const BodyPart &bodyPart, 
      const cv::Point2f &j0, const cv::Point2f &j1, const cv::Mat &imgMat, 
      const uint32_t minHessian, const std::vector <cv::KeyPoint> &keyPoints) 
      const;
    LimbLabel generateLabel(const BodyPart &bodyPart, Frame *frame, 
      const cv::Point2f &j0, const cv::Point2f &j1, 
      DetectorHelper *detectorHelper, std::map <std::string, float> params) 
      const;
    float compare(const BodyPart &bodyPart, const PartModel &model, 
      const cv::Point2f &j0, const cv::Point2f &j1, const float knnMatchCoeff) 
      const;
  };
}

#endif  // _LIBPOSE_SURFDETECTOR_HPP_ctujly
