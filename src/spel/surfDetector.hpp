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
  class SurfDetector : public Detector
  {
  protected:
    struct PartModel
    {
      POSERECT <cv::Point2f> partModelRect;
      std::vector <cv::KeyPoint> keyPoints;
      cv::Mat descriptors;
    };
  public:
    SurfDetector(void);
    virtual ~SurfDetector(void);
    virtual int getID(void) const;
    virtual void setID(int _id);
    virtual void train(const std::vector <Frame*> &_frames, std::map <std::string, float>);
    virtual std::map <uint32_t, std::vector <LimbLabel> > detect(Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels);
    virtual std::map <uint32_t, std::map <uint32_t, PartModel>> getPartModels(void);
    virtual std::map <uint32_t, std::map <uint32_t, std::vector <PartModel>>> getLabelModels(void);

  private:
#ifdef DEBUG
    FRIEND_TEST(surfDetectorTests, computeDescriptors);
    FRIEND_TEST(surfDetectorTests, train);
    FRIEND_TEST(surfDetectorTests, compare);
    FRIEND_TEST(surfDetectorTests, generateLabel);
    //FRIEND_TEST(surfDetectorTests, detect);
#endif  // DEBUG
    int id;
  protected:
    uint32_t minHessian = 500;
    float useSURFdet = 1.0f;
    float knnMatchCoeff = 0.8f;
    std::vector <cv::KeyPoint> keyPoints;
    // Variables for score comparer
    BodyPart const *comparer_bodyPart = 0;
    PartModel *comparer_model = 0;
    cv::Point2f const *comparer_j0 = 0;
    cv::Point2f const *comparer_j1 = 0;

    std::map <uint32_t, std::map <uint32_t, PartModel>> partModels;
    std::map <uint32_t, std::map <uint32_t, std::vector <PartModel>>> labelModels;

    virtual std::map <uint32_t, PartModel> computeDescriptors(Frame *frame, uint32_t minHessian);
    virtual PartModel computeDescriptors(BodyPart bodyPart, cv::Point2f j0, cv::Point2f j1, cv::Mat imgMat, uint32_t minHessian, std::vector <cv::KeyPoint> keyPoints);
    virtual LimbLabel generateLabel(const BodyPart &bodyPart, const Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1);
    virtual float compare(BodyPart bodyPart, PartModel model, cv::Point2f j0, cv::Point2f j1);
    virtual float compare(void);
  };

}

#endif  // _LIBPOSE_SURFDETECTOR_HPP_ctujly
