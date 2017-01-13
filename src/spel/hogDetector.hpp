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
  /// <summary>
  /// Helper class for <see cref="HogDetector" /> class.
  /// </summary>
  /// <seealso cref="DetectorHelper" />
  class HogDetectorHelper : public DetectorHelper
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="HogDetectorHelper"/> 
    /// class.
    /// </summary>
    HogDetectorHelper(void) ;
    /// <summary>
    /// Finalizes an instance of the <see cref="HogDetectorHelper"/> class.
    /// </summary>
    ~HogDetectorHelper(void) ;
  };  
  /// <summary>
  /// Implementation of the HoG method of detection.
  /// </summary>
  /// <seealso cref="Detector" />
  class HogDetector : public Detector
  {
  protected:    
    /// <summary>
    /// Part model helper struct.
    /// </summary>
    struct PartModel
    {      
      /// <summary>The part model rect.</summary>
      spelRECT <cv::Point2f> partModelRect;      
      /// <summary>The gradient strengths.</summary>
      std::vector <std::vector <std::vector <float>>> gradientStrengths;      
      /// <summary>The part image.</summary>
      cv::Mat partImage;
#ifdef DEBUG      
      /// <summary>The descriptors.</summary>
      std::vector<float> descriptors;
#endif  // DEBUG
    };
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="HogDetector"/> class.
    /// </summary>
    /// <param name="nbins">The nbins.</param>
    /// <param name="wndStride">The window stride.</param>
    /// <param name="padding">The padding.</param>
    /// <param name="blockSize">Size of the block.</param>
    /// <param name="blockStride">The block stride.</param>
    /// <param name="cellSize">Size of the cell.</param>
    /// <param name="wndSigma">The window sigma.</param>
    /// <param name="thresholdL2hys">The threshold l2hys.</param>
    /// <param name="gammaCorrection">
    /// if set to <c>true</c> then gamma correction is used.
    /// </param>
    /// <param name="nlevels">The nlevels.</param>
    /// <param name="derivAperture">The deriv aperture.</param>
    /// <param name="histogramNormType">
    /// Type of the histogram normalization.
    /// </param>
    HogDetector(
      uint8_t nbins = 9, 
      cv::Size wndStride = cv::Size(8, 8),
      cv::Size padding = cv::Size(32, 32),
      cv::Size blockSize = cv::Size(16, 16),
      cv::Size blockStride = cv::Size(8, 8),
      cv::Size cellSize = cv::Size(8, 8), 
      double wndSigma = -1,
      double thresholdL2hys = 0.2, 
      bool gammaCorrection = true,
      int nlevels = 64, 
      int derivAperture = 1,
      int histogramNormType = cv::HOGDescriptor::L2Hys) ;
    /// <summary>
    /// Finalizes an instance of the <see cref="HogDetector"/> class.
    /// </summary>
    ~HogDetector(void) ;
    /// <summary>Trains the specified frames.</summary>
    /// <param name="frames">The frames.</param>
    /// <param name="params">The parameters.</param>
    void train(const std::vector <Frame*> &frames, 
      std::map <std::string, float> params);
    /// <summary>Detects the specified frame.</summary>
    /// <param name="frame">The frame.</param>
    /// <param name="params">The parameters.</param>
    /// <param name="limbLabels">The limb labels.</param>
    /// <returns>The map of detected limb labels.</returns>
    std::map <uint32_t, std::vector <LimbLabel> > detect(Frame *frame, 
      std::map <std::string, float> params, const std::map <uint32_t, 
      std::vector <LimbLabel>> &limbLabels) const;
    /// <summary>Gets the part models.</summary>
    /// <returns>The part models.</returns>
    std::map <uint32_t, std::map <uint32_t, PartModel>> getPartModels(void) 
      const ;
    /// <summary>Gets the size of the cell.</summary>
    /// <returns>The size of the cell.</returns>
    cv::Size getCellSize(void) const ;
    /// <summary>Get the nbins.</summary>
    /// <returns>The nbins.</returns>
    uint8_t getnbins(void) const ;
    /// <summary>Calculates the hog.</summary>
    /// <param name="image">The image.</param>
    /// <param name="descriptors">The descriptors.</param>
    /// <param name="wndSize">Size of the window.</param>
    /// <param name="blockSize">Size of the block.</param>
    /// <param name="blockStride">The block stride.</param>
    /// <param name="cellSize">Size of the cell.</param>
    /// <param name="nbins">The nbins.</param>
    /// <param name="derivAperture">The deriv aperture.</param>
    /// <param name="wndSigma">The window sigma.</param>
    /// <param name="histogramNormType">Type of the histogram norm.</param>
    /// <param name="thresholdL2hys">The threshold l2hys.</param>
    /// <param name="gammaCorrection">The gamma correction.</param>
    /// <param name="nlevels">The nlevels.</param>
    /// <returns>The set of HoG descriptors.</returns>
    static std::vector <std::vector <std::vector <float>>> calculateHog(
      const cv::Mat &image, std::vector <float> &descriptors, 
      const cv::Size &wndSize, const cv::Size &blockSize, 
      const cv::Size &blockStride, const cv::Size &cellSize, 
      const int nbins, const int derivAperture, const double wndSigma, 
      const int histogramNormType, const double thresholdL2hys, 
      const bool gammaCorrection, const int nlevels);    
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
    /// <summary>The part size.</summary>
    std::map <uint32_t, cv::Size> m_partSize;
    /// <summary>The part models.</summary>
    std::map <uint32_t, std::map <uint32_t, PartModel>> m_partModels;
    /// <summary>The nbins.</summary>
    uint8_t m_nbins;
    /// <summary>The window stride.</summary>
    cv::Size m_wndStride;
    /// <summary>The padding.</summary>
    cv::Size m_padding;
    /// <summary>The block size.</summary>
    cv::Size m_blockSize;
    /// <summary>The block stride.</summary>
    cv::Size m_blockStride;
    /// <summary>The cell size.</summary>
    cv::Size m_cellSize;
    /// <summary>The window sigma.</summary>
    double m_wndSigma;
    /// <summary>The threshold l2hys.</summary>
    double m_thresholdL2hys;
    /// <summary>The gamma correction.</summary>
    bool m_gammaCorrection;
    /// <summary>The nlevels.</summary>
    int m_nlevels;
    /// <summary>The deriv aperture.</summary>
    int m_derivAperture;
    /// <summary>The histogram normalization type.</summary>
    int m_histogramNormType;
    /// <summary>The flag of gray images.</summary>
    bool m_bGrayImages = false;
    /// <summary>Generates the label.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="frame">The frame.</param>
    /// <param name="j0">The parent joint.</param>
    /// <param name="j1">The child joint.</param>
    /// <param name="detectorHelper">The detector helper.</param>
    /// <param name="params">The parameters.</param>
    /// <returns>The generated label.</returns>
    LimbLabel generateLabel(const BodyPart &bodyPart, Frame *frame, 
      const cv::Point2f &j0, const cv::Point2f &j1, 
      DetectorHelper *detectorHelper, 
      std::map <std::string, float> params) const;
    /// <summary>Gets the maximum width and height of the body parts.</summary>
    /// <param name="blockSize">Size of the block.</param>
    /// <param name="resizeFactor">The resize factor.</param>
    /// <returns>The maximum width and height of the body parts.</returns>
    std::map <uint32_t, cv::Size> getMaxBodyPartHeightWidth(
      const cv::Size &blockSize, const float resizeFactor) const;
    /// <summary>Computes the descriptors.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="j0">The parent joint.</param>
    /// <param name="j1">The child joint.</param>
    /// <param name="imgMat">The image.</param>
    /// <param name="wndSize">Size of the window.</param>
    /// <returns>The part model.</returns>
    PartModel computeDescriptors(const BodyPart &bodyPart, 
      const cv::Point2f &j0, const cv::Point2f &j1, 
      const cv::Mat &imgMat, const cv::Size &wndSize) const;
    /// <summary>Computes the descriptors.</summary>
    /// <param name="frame">The frame.</param>
    /// <returns>The descriptors.</returns>
    std::map <uint32_t, PartModel> computeDescriptors(Frame *frame) const;
    /// <summary>Compares the specified body part.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="partModel">The part model.</param>
    /// <param name="nbins">The nbins.</param>
    /// <returns>The comparison coefficient.</returns>
    float compare(const BodyPart &bodyPart, const PartModel &partModel, 
      const uint8_t nbins) const;
    /// <summary>Emplaces the default parameters.</summary>
    /// <param name="params">The parameters.</param>
    void emplaceDefaultParameters(
      std::map <std::string, float> &params) const ;
  };
}
#endif  // _LIBPOSE_HOGDETECTOR_HPP_
