#ifndef _LIBPOSE_SURFDETECTOR2_HPP_
#define _LIBPOSE_SURFDETECTOR2_HPP_

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
#include "spelParameters.hpp"

#ifdef DEBUG
#include <gtest/gtest.h>
#endif // DEBUG

namespace SPEL
{
  class SURFDetectorHelper: public DetectorHelper // It is gag
  {};

  class SURFDetector2: public Detector
  {
    FRIEND_TEST(SURFDetectorExperiments_B, SURFDetector2);
    FRIEND_TEST(SURFDetectorExperiments, getPartPolygon);
    //FRIEND_TEST(SURFDetectorExperiments, PartCellIndex);
  protected:
    struct SkeletonModel
    {
      void clear(void);
      ~SkeletonModel();
      std::vector<cv::KeyPoint> Keypoints; // All keypoints from the studied skeletons
      cv::Mat Descriptors; // All descriptors from the studied skeletons

      std::map<int, cv::Size> PartCellsCount;
      std::map<int, std::vector<int>> PartKeypoints;
      std::vector<int> StudiedFramesID;
    }; 

    struct Parameters
    {
      float minHessian = COMMON_SURF_DETECTOR_PARAMETERS::MIN_HESSIAN().second;
      float searchDistCoeff = DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT().second;
      float minTheta = DETECTOR_DETECT_PARAMETERS::MIN_THETA().second;
      float maxTheta = DETECTOR_DETECT_PARAMETERS::MAX_THETA().second;
      float stepTheta = DETECTOR_DETECT_PARAMETERS::STEP_THETA().second;
      float uniqueLocationCandidates = DETECTOR_DETECT_PARAMETERS::UNIQUE_LOCATION_CANDIDATES_COEFFICIENT().second;
      float uniqueAngleCandidates = DETECTOR_DETECT_PARAMETERS::UNIQUE_ANGLE_CANDIDATES_COEFFICIENT().second;
      int externalFrameHeight = static_cast<int>(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().second); // detector::maxFrameHeight is external height for SURFDetectorB
      int internalFrameHeight = 0;
      /*bool adjustSolves = false; // if "true" then solves will be returned in "adjusted Frame" scale else in "maxFrameHeight" coordinates 
      //needed Skeleton::getScale() for using this parameter  */

      float isWeakThreshold = DETECTOR_DETECT_PARAMETERS::IS_WEAK_THRESHOLD().second;
      float searchStepCoeff = DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT().second;

      float markingLinearError = 10.0f;
      float minCellSize = 2.0f;
      int FixedWidthCells = 0; // if >=1 then used equal cellsCount for all BodyParts 
      int FixedLenghtCells = 0; // if >=1 then used equal cellsCount for all BodyParts 
      bool useDefaultCellsCount = true;

      bool useMask = true;
    };

  public:
      SURFDetector2(void);
      SURFDetector2(std::map<std::string, float> params);
      ~SURFDetector2(void);
      void setParameters(std::map<std::string, float> params);
      void changeParameters(std::map<std::string, float> params) const;
      Frame* preparedFrame(Frame* frame) const;
      void setAutoInternalFrameHeight(std::vector<Frame*> frames);
      void SingleFrameTrain(Frame* frame_);
      std::vector<int> getStudiedFramesID(void) const;

      void Train(std::vector<Frame*> frames);
      void train(const std::vector<Frame*> &frames, std::map<std::string, float> params);
      std::map<uint32_t, std::vector<LimbLabel>> generateLimbLabels(Skeleton approximation) const;
      std::map<uint32_t, std::vector<LimbLabel>> Detect(Frame* frame_, std::map<uint32_t, std::vector<LimbLabel>> &Labels) const;
      std::map<uint32_t, std::vector<LimbLabel>> detect(Frame* frame,
          std::map<std::string, float> params,
         const std::map<uint32_t, std::vector<LimbLabel>> &limbLabels) const;

      std::vector<cv::KeyPoint> getPartKeypoints(int partID) const;
      std::map<uint32_t, std::vector<cv::KeyPoint>> getPartsKeypoints() const;

  private:
    const int partCellsLimit = 100000;
    void setCellsCount(std::map<int, std::vector<cv::Point2f>> &partPolygons, float markingError);
    void setFixedCellsCount(cv::Size partCellsCount);
    void setDefaultCellsCount(void);
    void correctingCellsSize(std::map<int, std::vector<cv::Point2f>> &partPolygons);
    std::vector<cv::KeyPoint> detectKeypoints(Frame* frame, bool useMask = true) const;
    mutable Parameters parameters;
    mutable SkeletonModel Trained;
    mutable SkeletonModel Local;
    
    // Moved to "spelGeometry.cpp":
    /*float getLenght(std::vector<cv::Point2f> polygon) const;
    float getWidth(std::vector<cv::Point2f> polygon) const;
    std::vector<cv::Point2f> getPartPolygon(float LWRatio, cv::Point2f p0, cv::Point2f p1) const;
    std::map<int, std::vector<cv::Point2f>> getAllPolygons(Skeleton &skeleton) const;
    */
    
    std::vector<int> PolygonsPriority(std::map<int, std::vector<cv::Point2f>> partRects) const;
    std::vector<cv::KeyPoint> SelectMaskKeypoints(cv::Mat &mask, std::vector<cv::KeyPoint> FrameKeypoints) const;
    int PartCellIndex(int PartID, cv::Point2f pt, std::vector<cv::Point2f> polygon, cv::Size CellsCount = cv::Size(0, 0)) const;
    int PartIndex(int CellIndex) const;

    LimbLabel generateLabel(const BodyPart &bodyPart,
        Frame *workFrame, const cv::Point2f &parent, const cv::Point2f &child,
        DetectorHelper *detectorHelper, std::map<std::string, float> params) const;  // It is gag
  };

}

#endif _LIBPOSE_SURFDETECTOR2_HPP_

