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
    DetectorHelper(void) noexcept;
    virtual ~DetectorHelper(void) noexcept;
  };

  class Detector
  {
  public:
    Detector(void) noexcept;
    virtual ~Detector(void) noexcept;
    virtual int getID(void) const noexcept = 0;
    virtual void setID(const int &_id) = 0;
    virtual void train(const std::vector <Frame*> &frames, std::map <std::string, float> params) = 0;
    virtual std::map <uint32_t, std::vector <LimbLabel> > detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels) const = 0;
    virtual std::map <uint32_t, std::vector <LimbLabel> > detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels, DetectorHelper *detectorHelper) const;
    virtual std::map <uint32_t, std::vector <LimbLabel>> merge(const std::map <uint32_t, std::vector <LimbLabel>> &first, const std::map <uint32_t, std::vector <LimbLabel>> &second, const std::map <uint32_t, std::vector <LimbLabel>> &secondUnfiltered) const;
    virtual void setDebugLevel(const uint8_t &_debugLevel) noexcept;
    virtual uint8_t getDebugLevel(void) const noexcept;
  protected:
#ifdef DEBUG
    FRIEND_TEST(DetectorTests, getFrame);
#endif  // DEBUG
    std::vector <Frame*> frames;
    uint32_t maxFrameHeight;
    uint8_t debugLevel = 1;

    virtual Frame *getFrame(const int32_t &frameId) const noexcept;
    virtual float getBoneLength(const cv::Point2f &begin, const cv::Point2f &end) const noexcept;
    virtual float getBoneWidth(const float &length, const BodyPart &bodyPart) const noexcept;
    virtual POSERECT <cv::Point2f> getBodyPartRect(const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1) const noexcept;
    virtual POSERECT <cv::Point2f> getBodyPartRect(const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1, const cv::Size &blockSize) const noexcept;
    virtual cv::Mat rotateImageToDefault(const cv::Mat &imgSource, const POSERECT <cv::Point2f> &initialRect, const float &angle, const cv::Size &size) const;
    virtual LimbLabel generateLabel(const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1, const std::string &detectorName, float _usedet, std::function<float()> compare) const;
    virtual LimbLabel generateLabel(const BodyPart &bodyPart, const Frame *workFrame, const cv::Point2f &p0, const cv::Point2f &p1, DetectorHelper *detectorHelper, std::map <std::string, float> params) const = 0;
    virtual LimbLabel generateLabel(const float &boneLength, const float &rotationAngle, const float &x, const float &y, const BodyPart &bodyPart, const Frame *workFrame, DetectorHelper *detectorHelper, std::map <std::string, float> params) const;
    virtual std::vector <LimbLabel> filterLimbLabels(const std::vector <LimbLabel> &sortedLabels, const float &uniqueLocationCandidates, const float &uniqueAngleCandidates) const;    
  };
}
#endif  // _LIBPOSE_DETECTOR_HPP_
