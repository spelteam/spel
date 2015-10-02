#ifndef _LIBPOSE_DETECTOR_HPP_
#define _LIBPOSE_DETECTOR_HPP_

// SPEL definitions
#include "predef.hpp"
#include "spelParameters.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// STL
#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif  // WINDOWS
#include <vector>
#include <map>
#include <string>
#include <exception>
#include <functional>

#include "frame.hpp"
#include "limbLabel.hpp"
#include "spelHelper.hpp"
#include "sequence.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"

namespace SPEL
{
  class DetectorHelper
  {
  public:
    DetectorHelper(void);
    virtual ~DetectorHelper(void);
  };

  class Detector
  {
  public:
    Detector(void) noexcept;
    virtual ~Detector(void) noexcept;
    virtual int getID(void) const = 0;
    virtual void setID(int _id) = 0;
    virtual void train(const std::vector <Frame*> &frames, std::map <std::string, float> params) = 0;
    virtual std::map <uint32_t, std::vector <LimbLabel> > detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) = 0;
    virtual std::map <uint32_t, std::vector <LimbLabel> > detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels, DetectorHelper *detectorHelper);
    virtual std::map <uint32_t, std::vector <LimbLabel>> merge(const std::map <uint32_t, std::vector <LimbLabel>> &first, const std::map <uint32_t, std::vector <LimbLabel>> &second, const std::map <uint32_t, std::vector <LimbLabel>> &secondUnfiltered);
  protected:
#ifdef DEBUG
    FRIEND_TEST(DetectorTests, getFrame);
#endif  // DEBUG
    std::vector <Frame*> frames;
    uint32_t maxFrameHeight;
    uint8_t debugLevelParam = 0;
    virtual Frame *getFrame(const int32_t &frameId) noexcept;
    virtual float getBoneLength(const cv::Point2f &begin, const cv::Point2f &end) noexcept;
    virtual float getBoneWidth(const float &length, const BodyPart &bodyPart) noexcept;
    virtual POSERECT <cv::Point2f> getBodyPartRect(const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1) noexcept;
    virtual POSERECT <cv::Point2f> getBodyPartRect(const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1, const cv::Size &blockSize) noexcept;
    virtual cv::Mat rotateImageToDefault(const cv::Mat &imgSource, const POSERECT <cv::Point2f> &initialRect, const float &angle, const cv::Size &size);
    virtual LimbLabel generateLabel(const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1, const std::string &detectorName, float _usedet, std::function<float()> compare);
    virtual LimbLabel generateLabel(const BodyPart &bodyPart, const Frame *workFrame, const cv::Point2f &p0, const cv::Point2f &p1, DetectorHelper *detectorHelper, std::map <std::string, float> params) = 0;
    virtual LimbLabel generateLabel(float boneLength, float rotationAngle, float x, float y, BodyPart bodyPart, Frame *workFrame, DetectorHelper *detectorHelper, std::map <std::string, float> params);
    virtual std::vector <LimbLabel> filterLimbLabels(std::vector <LimbLabel> &sortedLabels, float uniqueLocationCandidates, float uniqueAngleCandidates);
  };
}
#endif  // _LIBPOSE_DETECTOR_HPP_
