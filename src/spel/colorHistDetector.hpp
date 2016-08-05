#ifndef _LIBPOSE_COLORHISTDETECTOR_HPP_
#define _LIBPOSE_COLORHISTDETECTOR_HPP_

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
  /// <summary>
  /// Helper class for <see cref="ColorHistDetector" /> class.
  /// </summary>
  /// <seealso cref="DetectorHelper" />
  class ColorHistDetectorHelper : public DetectorHelper
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="ColorHistDetectorHelper"/> class.
    /// </summary>
    ColorHistDetectorHelper(void) noexcept;
    /// <summary>
    /// Finalizes an instance of the <see cref="ColorHistDetectorHelper"/> class.
    /// </summary>
    /// <returns></returns>
    virtual ~ColorHistDetectorHelper(void) noexcept;
    /// <summary>The pixel labels.</summary>
    std::map <int32_t, cv::Mat> pixelLabels;
  };
  /// <summary>
  /// Implementation of the Color Histogram method of detection.
  /// </summary>
  /// <seealso cref="Detector" />
  class ColorHistDetector : public Detector
  {
  protected:
    /// <summary>
    /// Part model helper struct.
    /// </summary>
    struct PartModel
    {
      /// <summary>
      /// Initializes a new instance of the <see cref="PartModel"/> struct.
      /// </summary>
      /// <param name="_nBins">The _n bins.</param>
      PartModel(uint8_t _nBins = 8);
      /// <summary>
      /// Finalizes an instance of the <see cref="PartModel"/> class.
      /// </summary>
      virtual ~PartModel(void);
      /// <summary>The nbins parameter.</summary>
      uint8_t nBins;
      /// <summary>The part histogram.</summary>
      std::vector <std::vector <std::vector <float>>> partHistogram;
      /// <summary>The background histogram.</summary>
      std::vector <std::vector <std::vector <float>>> bgHistogram;
      /// <summary>The size of foreground histogram.</summary>
      uint32_t sizeFG;
      /// <summary>The size of background histogram.</summary>
      uint32_t sizeBG;
      /// <summary>The foreground number samples.</summary>
      uint32_t fgNumSamples;
      /// <summary>The background number samples.</summary>
      uint32_t bgNumSamples;
      /// <summary>The foreground sample sizes.</summary>
      std::vector <uint32_t> fgSampleSizes;
      /// <summary>The background sample sizes.</summary>
      std::vector <uint32_t> bgSampleSizes;
      /// <summary>The foreground blank sizes.</summary>
      std::vector <uint32_t> fgBlankSizes;
      /// <summary>Copy operator.</summary>
      /// <param name="model">The model.</param>
      /// <returns></returns>
      virtual PartModel &operator=(const PartModel &model) noexcept;
      /// <summary>Calculates the factor.</summary>
      /// <returns>The factor.</returns>
      virtual uint8_t calculateFactor(void) const;
      /// <summary>Computes the pixel belonging likelihood.</summary>
      /// <param name="r">The r value.</param>
      /// <param name="g">The g value.</param>
      /// <param name="b">The b value.</param>
      /// <returns>
      /// The relative frequency of the RGB-color reiteration.
      /// </returns>
      virtual float computePixelBelongingLikelihood(const uint8_t r, 
        const uint8_t g, const uint8_t b) const;
      /// <summary>Sets the part histogram.</summary>
      /// <param name="partColors">The part colors.</param>
      virtual void setPartHistogram(
        const std::vector <cv::Point3i> &partColors);
      /// <summary>Adds the part histogram.</summary>
      /// <param name="partColors">The part colors.</param>
      /// <param name="nBlankPixels">The blank pixel count.</param>
      virtual void addPartHistogram(
        const std::vector <cv::Point3i> &partColors, 
        const uint32_t nBlankPixels);
      /// <summary>Gets the foreground average sample size.</summary>
      /// <returns>The foreground average sample size.</returns>
      virtual float getAvgSampleSizeFg(void) const;
      /// <summary>Gets the foreground average sample size between two samples.</summary>
      /// <param name="s1">The first sample.</param>
      /// <param name="s2">The second sample.</param>
      /// <returns>The foreground average sample size.</returns>
      virtual float getAvgSampleSizeFgBetween(const uint32_t s1,
        const uint32_t s2) const;
      /// <summary>Matches the part histograms.</summary>
      /// <param name="partModelPrev">The previous part model.</param>
      /// <returns>The euclidian distance between two histograms.</returns>
      virtual float matchPartHistogramsED(
        const PartModel &partModelPrev) const;
      /// <summary>Adds the background histogram.</summary>
      /// <param name="bgColors">The background colors.</param>
      virtual void addBackgroundHistogram(
        const std::vector <cv::Point3i> &bgColors);
    };
  public:
    /// <summary>
    /// Initializes a new instance of the 
    /// <see cref="ColorHistDetector"/> class.
    /// Default is 8 for 32 bit colourspace.
    /// </summary>
    /// <param name="_nBins">The nbins.</param>
    ColorHistDetector(uint8_t _nBins = 8);
    /// <summary>
    /// Finalizes an instance of the <see cref="ColorHistDetector"/> class.
    /// </summary>
    /// <returns></returns>
    virtual ~ColorHistDetector(void) noexcept;
    /// <summary>Trains the specified frames.</summary>
    /// <param name="_frames">The frames.</param>
    /// <param name="params">The parameters.</param>
    void train(const std::vector <Frame*> &_frames, 
      std::map <std::string, float> params);
    /// <summary>Detects the specified frame.</summary>
    /// <param name="frame">The frame.</param>
    /// <param name="params">The parameters.</param>
    /// <param name="limbLabels">The limb labels.</param>
    /// <returns>The map of detected limb labels.</returns>
    std::map <uint32_t, std::vector <LimbLabel> > detect(
      Frame *frame, std::map <std::string, float> params, 
      const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const;
    /// <summary>Gets the nbins.</summary>
    /// <returns>The nbins.</returns>
    uint8_t getNBins(void) const noexcept;
    /// <summary>Gets the frames.</summary>
    /// <returns>The frames.</returns>
    std::vector <Frame*> getFrames(void) const noexcept;
    /// <summary>Copy operator.</summary>
    /// <param name="c">The <see cref="ColorHistDetector" /> class instance.</param>
    /// <returns>The <see cref="ColorHistDetector" /> class instance.</returns>
    ColorHistDetector &operator=(const ColorHistDetector &c) noexcept;
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
    FRIEND_TEST(ColorHistDetectorTest, CalculateFactor);
    FRIEND_TEST(ColorHistDetectorTest, compare);
    FRIEND_TEST(DetectorTests, detect);
    FRIEND_TEST(DetectorTests, generateLabel1);
    FRIEND_TEST(DetectorTests, generateLabel2);
#endif  // DEBUG
    /// <summary>The nbins.</summary>
    const uint8_t nBins; 
    /// <summary>The part models.</summary>
    std::map <int32_t, PartModel> partModels;
    /// <summary>Builds the pixel distributions.</summary>
    /// <param name="frame">The frame.</param>
    /// <returns>The map of pixel distibutions.</returns>
    std::map <int32_t, cv::Mat> buildPixelDistributions(
      Frame *frame) const;
    /// <summary>Builds the pixel labels.</summary>
    /// <param name="frame">The frame.</param>
    /// <param name="pixelDistributions">The pixel distributions.</param>
    /// <returns>The pixel labels.</returns>
    std::map <int32_t, cv::Mat> buildPixelLabels(Frame *frame,
      const std::map <int32_t, cv::Mat> &pixelDistributions) const;
    /// <summary>Generates the label.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="frame">The frame.</param>
    /// <param name="j0">The first joint identifier.</param>
    /// <param name="j1">The second joint identifier.</param>
    /// <param name="detectorHelper">The detector helper.</param>
    /// <param name="params">The parameters.</param>
    /// <returns>The label.</returns>
    LimbLabel generateLabel(const BodyPart &bodyPart, 
      Frame *frame, const cv::Point2f &j0, const cv::Point2f &j1, 
      DetectorHelper *detectorHelper, std::map <std::string, 
      float> params) const;
    /// <summary>Compares the specified body part.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="frame">The frame.</param>
    /// <param name="pixelLabels">The pixel labels.</param>
    /// <param name="j0">The first joint identifier.</param>
    /// <param name="j1">The second joint identifier.</param>
    /// <returns>The comparison coefficient.</returns>
    float compare(const BodyPart &bodyPart, Frame *frame, 
      const std::map <int32_t, cv::Mat> &pixelLabels, const cv::Point2f &j0, 
      const cv::Point2f &j1) const;
  };
}
#endif  // _LIBPOSE_COLORHISTDETECTOR_HPP_
