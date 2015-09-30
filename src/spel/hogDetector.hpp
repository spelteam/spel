#ifndef _LIBPOSE_HOGDETECTOR_HPP_
#define _LIBPOSE_HOGDETECTOR_HPP_

// SPEL definitions
#include "predef.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// OpenCV
#include <opencv2/opencv.hpp>

#include "detector.hpp"

namespace SPEL
{
  class HogDetector : public Detector
  {
  protected:
    struct PartModel
    {
      POSERECT <cv::Point2f> partModelRect;
      std::vector <std::vector <std::vector <float>>> gradientStrengths;
      cv::Mat partImage;
#ifdef DEBUG
      std::vector<float> descriptors;
#endif  // DEBUG
    };
  public:
    HogDetector(void);
    virtual ~HogDetector(void);
    virtual int getID(void) const;
    virtual void setID(int _id);
    virtual void train(const std::vector <Frame*> &_frames, std::map <std::string, float> params);
    virtual std::map <uint32_t, std::vector <LimbLabel>> detect(Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels);
    virtual std::map <uint32_t, std::map <uint32_t, std::vector <PartModel>>> getLabelModels(void);
    virtual std::map <uint32_t, std::map <uint32_t, PartModel>> getPartModels(void);

    virtual cv::Size getCellSize(void);
    virtual uint8_t getnbins(void);
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
  protected:
    const uint8_t nbins = 9;
    std::map <uint32_t, cv::Size> partSize;
    std::map <uint32_t, std::map <uint32_t, PartModel>> partModels;
    std::map <uint32_t, std::map <uint32_t, std::vector <PartModel>>> labelModels;
    bool bGrayImages = false;
    float useHoGdet = 1.0f;
    //TODO(Vitaliy Koshura): Make some of them as detector params
    cv::Size blockSize = cv::Size(16, 16);
    cv::Size blockStride = cv::Size(8, 8);
    cv::Size cellSize = cv::Size(8, 8);
    cv::Size wndSize = cv::Size(64, 128);
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    cv::Size wndStride = cv::Size(8, 8);
    cv::Size padding = cv::Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = cv::HOGDescriptor::L2Hys;
    // Variables for score comparer
    BodyPart const *comparer_bodyPart = 0;
    PartModel *comparer_model = 0;

    virtual LimbLabel generateLabel(const BodyPart &bodyPart, const Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1);

    virtual std::map <uint32_t, cv::Size> getMaxBodyPartHeightWidth(std::vector <Frame*> frames, cv::Size blockSize, float resizeFactor);
    virtual PartModel computeDescriptors(BodyPart bodyPart, cv::Point2f j0, cv::Point2f j1, cv::Mat imgMat, int nbins, cv::Size wndSize, cv::Size blockSize, cv::Size blockStride, cv::Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType);
    virtual std::map <uint32_t, PartModel> computeDescriptors(Frame *frame, int nbins, cv::Size blockSize, cv::Size blockStride, cv::Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels, int derivAperture, int histogramNormType);
    virtual float compare(BodyPart bodyPart, PartModel partModel, uint8_t nbins);
    virtual float compare(void);
  };
}
#endif  // _LIBPOSE_HOGDETECTOR_HPP_
