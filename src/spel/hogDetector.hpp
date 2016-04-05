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
  class HogDetectorHelper : public DetectorHelper
  {
  public:
    HogDetectorHelper(void) noexcept;
    virtual ~HogDetectorHelper(void) noexcept;
  };

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
    HogDetector(void) noexcept;
    virtual ~HogDetector(void) noexcept;
    virtual void train(const std::vector <Frame*> &_frames, std::map <std::string, float> params);
    virtual std::map <uint32_t, std::vector <LimbLabel> > detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const;
    virtual std::map <uint32_t, std::map <uint32_t, PartModel>> getPartModels(void) const noexcept;

    virtual cv::Size getCellSize(void) const noexcept;
    virtual uint8_t getnbins(void) const noexcept;

    static std::vector <std::vector <std::vector <float>>> calculateHog(const cv::Mat &image, std::vector <float> &descriptors, const cv::Size &wndSize, const cv::Size &blockSize, const cv::Size &blockStride, const cv::Size &cellSize, const int nbins, const int derivAperture, const double wndSigma, const int histogramNormType, const double thresholdL2hys, const bool gammaCorrection, const int nlevels);

  private:
#ifdef DEBUG
    FRIEND_TEST(HOGDetectorTests, computeDescriptor);
    FRIEND_TEST(HOGDetectorTests, computeDescriptors);
    FRIEND_TEST(HOGDetectorTests, getMaxBodyPartHeightWidth);
    FRIEND_TEST(HOGDetectorTests, train);
    FRIEND_TEST(HOGDetectorTests, generateLabel);
    FRIEND_TEST(HOGDetectorTests, detect);
    FRIEND_TEST(HOGDetectorTests, compare);
    FRIEND_TEST(HOGDetectorTests, getPartModels);
    FRIEND_TEST(HOGDetectorTests, getCellSize);
    FRIEND_TEST(HOGDetectorTests, getNBins);
    FRIEND_TEST(HOGDetectorTests, calculateHog);
#endif  // DEBUG
  protected:
    const uint8_t nbins = 9;
    std::map <uint32_t, cv::Size> partSize;
    std::map <uint32_t, std::map <uint32_t, PartModel>> partModels;
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

    virtual LimbLabel generateLabel(const BodyPart &bodyPart, const Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1, DetectorHelper *detectorHelper, std::map <std::string, float> params) const;
    virtual std::map <uint32_t, cv::Size> getMaxBodyPartHeightWidth(std::vector <Frame*> frames, cv::Size blockSize, float resizeFactor) const;
    virtual PartModel computeDescriptors(const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1, const cv::Mat &imgMat, const int nbins, const cv::Size &wndSize, const cv::Size &blockSize, const cv::Size &blockStride, const cv::Size &cellSize, const double wndSigma, const double thresholdL2hys, const bool gammaCorrection, const int nlevels, const int derivAperture, const int histogramNormType, const bool bGrayImages) const;
    virtual std::map <uint32_t, PartModel> computeDescriptors(const Frame *frame, const int nbins, const cv::Size &blockSize, const cv::Size &blockStride, const cv::Size &cellSize, const double wndSigma, const double thresholdL2hys, const bool gammaCorrection, const int nlevels, const int derivAperture, const int histogramNormType, const bool bGrayImages) const;
    virtual float compare(const BodyPart &bodyPart, const PartModel &partModel, const uint8_t nbins) const;
  };
}
#endif  // _LIBPOSE_HOGDETECTOR_HPP_
