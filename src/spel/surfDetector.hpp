#ifndef _LIBPOSE_SURFDETECTOR_HPP_
#define _LIBPOSE_SURFDETECTOR_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <utility>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#if !defined (HAVE_OPENCV_XFEATURES2D)
#error "Unsupported version of OpenCV. XFeatures2D are not installed."
#endif
#include "opencv2/features2d.hpp"
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "detector.hpp"

namespace SPEL
{  
  /// <summary>
  /// Helper class for <see cref="SurfDetector" /> class.
  /// </summary>
  /// <seealso cref="DetectorHelper" />
  class SurfDetectorHelper : public DetectorHelper
  {
  public:
    /// <summary>
    /// Initializes a new instance of the 
    /// <see cref="SurfDetectorHelper"/> class.
    /// </summary>
    SurfDetectorHelper(void) noexcept;
    /// <summary>
    /// Finalizes an instance of the <see cref="SurfDetectorHelper"/> class.
    /// </summary>
    virtual ~SurfDetectorHelper(void) noexcept;    
    /// <summary>The key points.</summary>
    std::vector <cv::KeyPoint> keyPoints;
  };  
  /// <summary>
  /// Implementation of the SURF method of detection.
  /// </summary>
  /// <seealso cref="Detector" />
  class SurfDetector : public Detector
  {
  protected:    
    /// <summary>
    /// Part model helper struct.
    /// </summary>
    struct PartModel
    {
      /// <summary>
      /// Finalizes an instance of the <see cref="PartModel"/> class.
      /// </summary>
      virtual ~PartModel(void);      
      /// <summary>The part model rect.</summary>
      POSERECT <cv::Point2f> partModelRect;      
      /// <summary>The key points.</summary>
      std::vector <cv::KeyPoint> keyPoints;      
      /// <summary>The descriptors.</summary>
      cv::Mat descriptors;
    };
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="SurfDetector"/> class.
    /// </summary>
    SurfDetector(void) noexcept;
    /// <summary>
    /// Finalizes an instance of the <see cref="SurfDetector"/> class.
    /// </summary>
    virtual ~SurfDetector(void) noexcept;
    /// <summary>Trains the specified frames.</summary>
    /// <param name="_frames">The frames.</param>
    /// <param name="">The parameters.</param>
    void train(const std::vector <Frame*> &frames, 
      std::map <std::string, float> params);
    /// <summary>Detects the specified frame.</summary>
    /// <param name="frame">The frame.</param>
    /// <param name="params">The parameters.</param>
    /// <param name="limbLabels">The limb labels.</param>
    /// <returns>The map of detected limb labels.</returns>
    std::map <uint32_t, std::vector <LimbLabel> > detect(Frame *frame, 
      std::map <std::string, float> params, 
      const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const;
    /// <summary>Gets the part models.</summary>
    /// <returns>The part models.</returns>
    std::map <uint32_t, std::map <uint32_t, PartModel>> getPartModels(void) 
      const noexcept;
  private:
#ifdef DEBUG
    FRIEND_TEST(surfDetectorTests, computeDescriptors);
    FRIEND_TEST(surfDetectorTests, train);
    FRIEND_TEST(surfDetectorTests, compare);
    FRIEND_TEST(surfDetectorTests, generateLabel);
    FRIEND_TEST(surfDetectorTests, PartModel);
    FRIEND_TEST(surfDetectorTests, getPartModels);
    //FRIEND_TEST(surfDetectorTests, detect);
#endif  // DEBUG
  protected:    
    /// <summary>The part models.</summary>
    std::map <uint32_t, std::map <uint32_t, PartModel>> partModels;
    /// <summary>Computes the descriptors.</summary>
    /// <param name="frame">The frame.</param>
    /// <param name="minHessian">The minimum hessian.</param>
    /// <returns>
    /// The map of part models with computed descriptors for each frame.
    /// </returns>
    std::map <uint32_t, PartModel> computeDescriptors(Frame *frame, 
      const uint32_t minHessian) const;
    /// <summary>Computes the descriptors.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="j0">The parent joint.</param>
    /// <param name="j1">The child joint.</param>
    /// <param name="imgMat">The image.</param>
    /// <param name="keyPoints">The key points.</param>
    /// <returns>The part model with computed desciptors.</returns>
    PartModel computeDescriptors(const BodyPart &bodyPart, 
      const cv::Point2f &j0, const cv::Point2f &j1, const cv::Mat &imgMat, 
      const std::vector <cv::KeyPoint> &keyPoints) const;
    /// <summary>Generates the label.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="frame">The frame.</param>
    /// <param name="j0">The parent joint.</param>
    /// <param name="j1">The child joint.</param>
    /// <param name="detectorHelper">The detector helper.</param>
    /// <param name="params">The parameters.</param>
    /// <returns>The generated limb label.</returns>
    LimbLabel generateLabel(const BodyPart &bodyPart, Frame *frame, 
      const cv::Point2f &j0, const cv::Point2f &j1, 
      DetectorHelper *detectorHelper, std::map <std::string, float> params) 
      const;
    /// <summary>Compares the specified body part.</summary>
    /// <param name="bodyPart">The body part.</param>
    /// <param name="model">The model.</param>
    /// <param name="j0">The parent joint.</param>
    /// <param name="j1">The child joint.</param>
    /// <param name="knnMatchCoeff">The KNN match coeff.</param>
    /// <returns>The comparison coefficient.</returns>
    float compare(const BodyPart &bodyPart, const PartModel &model, 
      const cv::Point2f &j0, const cv::Point2f &j1, const float knnMatchCoeff) 
      const;
  };
}

#endif  // _LIBPOSE_SURFDETECTOR_HPP_ctujly
