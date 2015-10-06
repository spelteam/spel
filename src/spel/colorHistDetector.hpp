#ifndef _LIBPOSE_COLORHISTDETECTOR_HPP_
#define _LIBPOSE_COLORHISTDETECTOR_HPP_

// SPEL definitions
#include "predef.hpp"
#include "spelParameters.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// OpenCV
#include <opencv2/opencv.hpp>

#include "detector.hpp"

namespace SPEL
{
  class ColorHistDetectorHelper : public DetectorHelper
  {
  public:
    ColorHistDetectorHelper(void) noexcept;
    virtual ~ColorHistDetectorHelper(void) noexcept;
    std::map <int32_t, cv::Mat> pixelDistributions;
    std::map <int32_t, cv::Mat> pixelLabels;
  };

  class ColorHistDetector : public Detector
  {
  protected:
    struct PartModel
    {
      PartModel(uint8_t _nBins = 8);
      uint8_t nBins;
      std::vector <std::vector <std::vector <float>>> partHistogram;
      std::vector <std::vector <std::vector <float>>> bgHistogram;
      uint32_t sizeFG;
      uint32_t sizeBG;
      uint32_t fgNumSamples;
      uint32_t bgNumSamples;
      std::vector <uint32_t> fgSampleSizes;
      std::vector <uint32_t> bgSampleSizes;
      std::vector <uint32_t> fgBlankSizes;
      virtual PartModel &operator=(const PartModel &model) noexcept;
      virtual uint8_t calculateFactor(void) const;
      virtual float computePixelBelongingLikelihood(const uint8_t &r, const uint8_t &g, const uint8_t &b) const;
      virtual void setPartHistogram(const std::vector <cv::Point3i> &partColors);
      virtual void addPartHistogram(const std::vector <cv::Point3i> &partColors, const uint32_t &nBlankPixels);
      virtual float getAvgSampleSizeFg(void) const;
      virtual float getAvgSampleSizeFgBetween(const uint32_t &s1, const uint32_t &s2) const;
      virtual float matchPartHistogramsED(const PartModel &partModelPrev) const;
      virtual void addBackgroundHistogram(const std::vector <cv::Point3i> &bgColors);
    };
  public:
    ColorHistDetector(uint8_t _nBins = 8);  // default is 8 for 32 bit colourspace
    virtual ~ColorHistDetector(void) noexcept;
    virtual int getID(void) const noexcept;
    virtual void setID(const int &_id) noexcept;
    virtual void train(const std::vector <Frame*> &_frames, std::map <std::string, float> params);
    virtual std::map <uint32_t, std::vector <LimbLabel> > detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const;
    virtual uint8_t getNBins(void) const noexcept;
    virtual std::vector <Frame*> getFrames(void) const noexcept;
    virtual ColorHistDetector &operator=(const ColorHistDetector &c) noexcept;
  private:
#ifdef DEBUG
    FRIEND_TEST(colorHistDetectorTest, Constructors);
    FRIEND_TEST(colorHistDetectorTest, computePixelBelongingLikelihood);
    FRIEND_TEST(colorHistDetectorTest, Operators);
    FRIEND_TEST(colorHistDetectorTest, setPartHistogram);
    FRIEND_TEST(colorHistDetectorTest, addpartHistogram);
    FRIEND_TEST(colorHistDetectorTest, getAvgSampleSizeFg);
    FRIEND_TEST(colorHistDetectorTest, getAvgSampleSizeFgBetween);
    FRIEND_TEST(colorHistDetectorTest, matchPartHistogramsED);
    FRIEND_TEST(colorHistDetectorTest, addBackgroundHistogram);
    FRIEND_TEST(colorHistDetectorTest, buildPixelDistributions);
    FRIEND_TEST(colorHistDetectorTest, BuildPixelLabels);
    FRIEND_TEST(colorHistDetectorTest, generateLabel);
    FRIEND_TEST(colorHistDetectorTest, detect);
    FRIEND_TEST(colorHistDetectorTest, Train);
#endif  // DEBUG
    int id;
  protected:
    const uint8_t nBins;
    std::map <int32_t, PartModel> partModels;
        
    virtual std::map <int32_t, cv::Mat> buildPixelDistributions(const Frame *frame) const;
    virtual std::map <int32_t, cv::Mat> buildPixelLabels(const Frame *frame, const std::map <int32_t, cv::Mat> &pixelDistributions) const;
    virtual LimbLabel generateLabel(const BodyPart &bodyPart, const Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1, DetectorHelper *detectorHelper, std::map <std::string, float> params) const;
    virtual float compare(const BodyPart &bodyPart, const Frame *frame, const std::map <int32_t, cv::Mat> &pixelDistributions, const std::map <int32_t, cv::Mat> &pixelLabels, const cv::Point2f &j0, const cv::Point2f &j1) const;
  };
}
#endif  // _LIBPOSE_COLORHISTDETECTOR_HPP_
