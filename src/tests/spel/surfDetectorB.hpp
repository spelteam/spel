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
      float minHessian = 300;
      float searchDistCoeff = 0.5f;
      float minTheta = 90.0f;
      float maxTheta = 100.0f;
      float stepTheta = 10.0f;
      float uniqueLocationCandidates = 0.1f;
      float uniqueAngleCandidates = 0.1f;

      float isWeakThreshold = 0.1f;
      float searchStepCoeff = 0.2f;

      float markingLinearError = 10.0f;
      float FixedWidthCells = 0.0f; // if FixedWidthCells >=1 && FixedLenghtCells >=1 then used equal cellsCount for all BodyParts 
      float FixedLenghtCells = 0.0f; //
      float useDefaultCellsCount = 1.0f; // bool
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
    void SURFDetector::setFixedCellsCount(cv::Size partCellsCount = cv::Size(3, 3));
    void SURFDetector::setDefaultCellsCount(void);
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
