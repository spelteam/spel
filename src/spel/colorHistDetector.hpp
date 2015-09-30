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
      virtual uint8_t calculateFactor(void);
      virtual float computePixelBelongingLikelihood(uint8_t r, uint8_t g, uint8_t b);
      virtual void setPartHistogram(const std::vector <cv::Point3i> &partColors);
      virtual void addPartHistogram(const std::vector <cv::Point3i> &partColors, uint32_t nBlankPixels);
      virtual float getAvgSampleSizeFg(void);
      virtual float getAvgSampleSizeFgBetween(uint32_t s1, uint32_t s2);
      virtual float matchPartHistogramsED(const PartModel &partModelPrev);
      virtual void addBackgroundHistogram(const std::vector <cv::Point3i> &bgColors);
    };
  public:
    ColorHistDetector(uint8_t _nBins = 8);  // default is 8 for 32 bit colourspace
    virtual ~ColorHistDetector(void) noexcept;
    virtual int getID(void) const noexcept;
    virtual void setID(int _id) noexcept;
    virtual void train(const std::vector <Frame*> &_frames, std::map <std::string, float> params);
    virtual std::map <uint32_t, std::vector <LimbLabel> > detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels);
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
    float useCSdet = 1.0f;
    std::map <int32_t, cv::Mat> pixelDistributions;
    std::map <int32_t, cv::Mat> pixelLabels;
        
    virtual std::map <int32_t, cv::Mat> buildPixelDistributions(const Frame *frame);
    virtual std::map <int32_t, cv::Mat> buildPixelLabels(const Frame *frame, const std::map <int32_t, cv::Mat> &pixelDistributions);
    virtual LimbLabel generateLabel(const BodyPart &bodyPart, const Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1);

    // Variables for score comparer
    BodyPart const *comparer_bodyPart = 0;
    Frame const **comparer_frame = 0;
    cv::Point2f const *comparer_j0 = 0;
    cv::Point2f const *comparer_j1 = 0;

    virtual float compare(void);
    virtual float compare(const BodyPart &bodyPart, const Frame *frame, const std::map <int32_t, cv::Mat> &pixelDistributions, const std::map <int32_t, cv::Mat> &pixelLabels, const cv::Point2f &j0, const cv::Point2f &j1);
  };
}
#endif  // _LIBPOSE_COLORHISTDETECTOR_HPP_
