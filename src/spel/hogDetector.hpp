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
    HogDetector(const uint8_t nbins = 9, 
      const cv::Size blockSize = cv::Size(16, 16), 
      const cv::Size blockStride = cv::Size(8, 8),
      const cv::Size cellSize = cv::Size(8, 8),
      const double wndSigma = -1,
      const double thresholdL2hys = 0.2,
      const bool gammaCorrection = true,
      const int nlevels = 64,
      const cv::Size wndStride = cv::Size(8, 8),
      const cv::Size padding = cv::Size(32, 32),
      const int derivAperture = 1,
      const int histogramNormType = cv::HOGDescriptor::L2Hys) noexcept;
    virtual ~HogDetector(void) noexcept;
    void train(const std::vector <Frame*> &_frames, 
      std::map <std::string, float> params);
    std::map <uint32_t, std::vector <LimbLabel> > detect(const Frame *frame, 
      std::map <std::string, float> params, 
      const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const;
    std::map <uint32_t, std::map <uint32_t, PartModel>> getPartModels(void) 
      const noexcept;
    std::vector <std::vector <std::vector <float>>> calculateHog(
      const cv::Mat &image, const cv::Size &wndSize,
      std::vector <float> &descriptors) const;
    cv::Size getCellSize(void) const noexcept;
    uint8_t getnbins(void) const noexcept;

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
    std::map <uint32_t, cv::Size> partSize;
    std::map <uint32_t, std::map <uint32_t, PartModel>> partModels;
    const uint8_t m_nbins;
    const cv::Size m_blockSize;
    const cv::Size m_blockStride;
    const cv::Size m_cellSize;
    const double m_wndSigma;
    const double m_thresholdL2hys;
    const bool m_gammaCorrection;
    const int m_nlevels;
    const cv::Size m_wndStride;
    const cv::Size m_padding;
    const int m_derivAperture;
    const int m_histogramNormType;

    LimbLabel generateLabel(const BodyPart &bodyPart, const Frame *frame, 
      const cv::Point2f &j0, const cv::Point2f &j1,
      DetectorHelper *detectorHelper, std::map <std::string, float> params) 
      const;
    std::map <uint32_t, cv::Size> getMaxBodyPartHeightWidth(
      const std::vector <Frame*> &frames, float resizeFactor) const;
    PartModel computeDescriptors(const BodyPart &bodyPart, 
      const cv::Point2f &j0, const cv::Point2f &j1, const cv::Mat &imgMat, 
      const cv::Size &wndSize, const bool bGrayImages) const;
    std::map <uint32_t, PartModel> computeDescriptors(const Frame *frame, 
      const bool bGrayImages) const;
    float compare(const BodyPart &bodyPart, const PartModel &partModel) const;
  };
}
#endif  // _LIBPOSE_HOGDETECTOR_HPP_
