#ifndef _LIBPOSE_SURFDETECTOR_HPP_
#define _LIBPOSE_SURFDETECTOR_HPP_

#include <opencv2/opencv.hpp>
#if OpenCV_VERSION_MAJOR == 2 && OpenCV_VERSION_MINOR == 4
#include <opencv2/nonfree/features2d.hpp>
#else
#error "Unsupported version of OpenCV. Should be 2.4.*.*"
#endif
#include "detector.hpp"

using namespace std;
using namespace cv;

class SurfDetector : public Detector
{
  public:
    SurfDetector(void);
    int getID(void);
    void setID(int _id);
    void train(vector <Frame*> _frames, map <string, float>);
    vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels);
  private:
    int id;
    uint8_t debugLevelParam;
    struct PartModel
    {
      POSERECT <Point2f> partModelRect;
      vector <KeyPoint> keyPoints;
      Mat descriptors;
    };
    map <uint32_t, map <uint32_t, PartModel>> partModels;
    map <uint32_t, PartModel> computeDescriptors(Frame *frame, uint32_t minHessian);
    PartModel computeDescriptors(BodyPart bodyPart, Point2f j0, Point2f j1, Mat imgMat, uint32_t minHessian);
    LimbLabel generateLabel(Frame *frame, BodyPart bodyPart, Point2f j0, Point2f j1, PartModel partModel, float _useSURFdet);
    float compare(BodyPart bodyPart, PartModel model);
};

#endif  // _LIBPOSE_SURFDETECTOR_HPP_

