#ifndef _LIBPOSE_SURFDETECTOR_HPP_
#define _LIBPOSE_SURFDETECTOR_HPP_

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
#warning "Unsupported version on OpenCV"
#include <opencv2/features2d/features2d.hpp>
#endif
#else
#error "Unsupported version on OpenCV"
#endif

#include "detector.hpp"

namespace SPEL
{
#if OpenCV_VERSION_MAJOR == 3 && defined (HAVE_OPENCV_XFEATURES2D)
  using namespace xfeatures2d;
#endif

  using namespace std;
  using namespace cv;

  class SurfDetector : public Detector
  {
  protected:
    struct PartModel
    {
      POSERECT <Point2f> partModelRect;
      vector <KeyPoint> keyPoints;
      Mat descriptors;
    };
  public:
    SurfDetector(void);
    int getID(void) const;
    void setID(int _id);
    void train(vector <Frame*> _frames, map <string, float>);
    map <uint32_t, vector <LimbLabel> > detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels);
    map <uint32_t, map <uint32_t, PartModel>> getPartModels(void);
    map <uint32_t, map <uint32_t, vector <PartModel>>> getLabelModels(void);

  private:
#ifdef DEBUG
    FRIEND_TEST(surfDetectorTests, computeDescriptors);
    FRIEND_TEST(surfDetectorTests, train);
    FRIEND_TEST(surfDetectorTests, compare);
    FRIEND_TEST(surfDetectorTests, generateLabel);
#endif  // DEBUG
    int id;
    uint32_t minHessian = 500;
    float useSURFdet = 1.0f;
    float knnMatchCoeff = 0.8f;
    vector <KeyPoint> keyPoints;
    // Variables for score comparer
    BodyPart *comparer_bodyPart = 0;
    PartModel *comparer_model = 0;
    Point2f *comparer_j0 = 0;
    Point2f *comparer_j1 = 0;

    map <uint32_t, map <uint32_t, PartModel>> partModels;
    map <uint32_t, map <uint32_t, vector <PartModel>>> labelModels;

    map <uint32_t, PartModel> computeDescriptors(Frame *frame, uint32_t minHessian);
    PartModel computeDescriptors(BodyPart bodyPart, Point2f j0, Point2f j1, Mat imgMat, uint32_t minHessian, vector <KeyPoint> keyPoints);
    LimbLabel generateLabel(BodyPart bodyPart, Frame *frame, Point2f j0, Point2f j1);
    float compare(BodyPart bodyPart, PartModel model, Point2f j0, Point2f j1);
    float compare(void);
  };

}

#endif  // _LIBPOSE_SURFDETECTOR_HPP_

