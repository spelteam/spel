#ifndef _LIBPOSE_HOGDETECTOR_HPP_
#define _LIBPOSE_HOGDETECTOR_HPP_

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

#include <opencv2/opencv.hpp>
#include "detector.hpp"

using namespace std;
using namespace cv;

class HogDetector : public Detector
{
  protected:
    struct PartModel
    {
      POSERECT <Point2f> partModelRect;
      vector <vector <vector <float>>> gradientStrengths;
      Mat partImage;
#ifdef DEBUG
      vector<float> descriptors;
#endif  // DEBUG
    };
  public:
    HogDetector(void);
    int getID(void) const;
    void setID(int _id);
    void train(vector <Frame*> _frames, map <string, float> params);
    vector <vector <LimbLabel>> detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels);
    map <uint32_t, map <uint32_t, vector <PartModel>>> getLabelModels(void);
    map <uint32_t, map <uint32_t, PartModel>> getPartModels(void);

    Size getCellSize(void);
    uint8_t getnbins(void);
  private:
#ifdef DEBUG
    FRIEND_TEST(HOGDetectorTests, computeDescriptor);
    FRIEND_TEST(HOGDetectorTests, computeDescriptors);
    FRIEND_TEST(HOGDetectorTests, getMaxBodyPartHeightWidth);
    FRIEND_TEST(HOGDetectorTests, train);
    FRIEND_TEST(HOGDetectorTests, generateLabel);
    FRIEND_TEST(HOGDetectorTests, detect);
    FRIEND_TEST(HOGDetectorTests, compare);
    FRIEND_TEST(HOGDetectorTests, getLabelModels);
    FRIEND_TEST(HOGDetectorTests, getPartModels);
    FRIEND_TEST(HOGDetectorTests, getCellSize);
    FRIEND_TEST(HOGDetectorTests, getNBins);
#endif  // DEBUG
    int id;
    Size savedCellSize;
    uint8_t savednbins;
    map <uint32_t, Size> partSize;
    map <uint32_t, map <uint32_t, PartModel>> partModels;
    map <uint32_t, map <uint32_t, vector <PartModel>>> labelModels;
    bool bGrayImages = false;
    float useHoGdet = 1.0f;
    //TODO(Vitaliy Koshura): Make some of them as detector params
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    Size wndSize = Size(64, 128);
    const uint8_t nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;
    // Variables for score comparer
    BodyPart *comparer_bodyPart = 0;
    PartModel *comparer_model = 0;

    LimbLabel generateLabel(BodyPart bodyPart, Frame *frame, Point2f j0, Point2f j1);

    map <uint32_t, Size> getMaxBodyPartHeightWidth(vector <Frame*> frames, Size blockSize, float resizeFactor);
    PartModel computeDescriptors(BodyPart bodyPart, Point2f j0, Point2f j1, Mat imgMat, int nbins, Size wndSize, Size blockSize, Size blockStride, Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType);
    map <uint32_t, PartModel> computeDescriptors(Frame *frame, int nbins, Size blockSize, Size blockStride, Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType);
    float compare(BodyPart bodyPart, PartModel partModel, uint8_t nbins);
    float compare(void);
};

#endif  // _LIBPOSE_HOGDETECTOR_HPP_
