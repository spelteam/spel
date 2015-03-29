#ifndef _LIBPOSE_HOGDETECTOR_HPP_
#define _LIBPOSE_HOGDETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include "detector.hpp"

using namespace std;
using namespace cv;

class HogDetector : public Detector
{
  public:
    HogDetector(void);
    int getID(void);
    void setID(int _id);
    void train(vector <Frame*> _frames, map <string, float> params);
    vector <vector <LimbLabel>> detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels);
  private:
    struct PartModel
    {
      POSERECT <Point2f> partModelRect;
      vector <float> descriptors;
    };
    int id;
    uint8_t debugLevelParam;
    map <uint32_t, Size> partSize;
    map <uint32_t, map <uint32_t, PartModel>> partModels;

    LimbLabel generateLabel(BodyPart bodyPart, Point2f j0, Point2f j1, PartModel descriptors, float _useHoGdet);

    map <uint32_t, Size> getMaxBodyPartHeightWidth(vector <Frame*> frames, Size blockSize);
    PartModel computeDescriptors(BodyPart bodyPart, Point2f j0, Point2f j1, Mat imgMat, int nbins, Size wndSize, Size blockSize, Size blockStride, Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType);
    map <uint32_t, PartModel> computeDescriptors(Frame *frame, int nbins, Size blockSize, Size blockStride, Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType);
    float compare(BodyPart bodyPart, PartModel partModel);
};

#endif  // _LIBPOSE_HOGDETECTOR_HPP_
