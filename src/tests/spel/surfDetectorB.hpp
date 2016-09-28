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

namespace SPEL
{
  class SURFDetectorHelper: public DetectorHelper // It is gag
  {};

  class SURFDetector: public Detector
  {
  protected:
    
    struct SkeletonModel
    {
      void clear(void);
      ~SkeletonModel();
      std::vector<cv::KeyPoint> Keypoints; // All keypoints from the studied skeletons
      cv::Mat Descriptors; // All descriptors from the studied skeletons

      std::map<int, cv::Size> PartCellsCount;
      std::map<int, int> PartKeypointsCount;
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

      float isWeakThreshold = DETECTOR_DETECT_PARAMETERS::IS_WEAK_THRESHOLD().second;
      float searchStepCoeff = DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT().second;

      float markingLinearError = 10.0f;
      int FixedWidthCells = 0; // if >=1 then used equal cellsCount for all BodyParts 
      int FixedLenghtCells = 0; // if >=1 then used equal cellsCount for all BodyParts 
      bool useDefaultCellsCount = true;
    };

  public:
      SURFDetector(void);
      ~SURFDetector(void);
      void setParameters(std::map<std::string, float> params);
      void changeParameters(std::map<std::string, float> params) const;
      void SingleFrameTrain(Frame* frame);
      std::vector<int> getStudiedFramesID(void) const;

      void Train(std::vector<Frame*> frames);
      void train(const std::vector<Frame*> &frames, std::map<std::string, float> params);

      std::map<uint32_t, std::vector<LimbLabel>> Detect(Frame* frame) const;
      std::map<uint32_t, std::vector<LimbLabel>> detect(Frame* frame,
          std::map<std::string, float> params,
         const std::map<uint32_t, std::vector<LimbLabel>> &limbLabels) const;

  private:
    void setCellsCount(std::map<int, std::vector<cv::Point2f>> &partPolygons, float markingError);
    void setFixedCellsCount(cv::Size partCellsCount);
    void setDefaultCellsCount(void);
    mutable Parameters parameters;
    mutable SkeletonModel Trained;

    float getLenght(std::vector<cv::Point2f> polygon) const;
    float getWidth(std::vector<cv::Point2f> polygon) const;
    std::vector<cv::Point2f> getPartPolygon(float LWRatio, cv::Point2f p0, cv::Point2f p1) const;

    std::map<int, std::vector<cv::Point2f>> getAllPolygons(Skeleton &skeleton) const;
    std::vector<int> PolygonsPriority(std::map<int, std::vector<cv::Point2f>> partRects) const;
    std::vector<cv::KeyPoint> SelectMaskKeypoints(cv::Mat &mask, std::vector<cv::KeyPoint> FrameKeypoints) const;
    int PartCellIndex(int PartID, cv::Point2f pt, std::vector<cv::Point2f> polygon, cv::Size CellsCount = cv::Size(1, 1)) const;
 
    LimbLabel generateLabel(const BodyPart &bodyPart,
        Frame *workFrame, const cv::Point2f &parent, const cv::Point2f &child,
        DetectorHelper *detectorHelper, std::map<std::string, float> params) const;  // It is gag
  };


}
