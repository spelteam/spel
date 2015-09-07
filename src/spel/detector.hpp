#ifndef _LIBPOSE_DETECTOR_HPP_
#define _LIBPOSE_DETECTOR_HPP_

// SPEL definitions
#include "predef.hpp"

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
  class Detector
  {
  public:
    Detector(void);
    virtual ~Detector(void);
    virtual int getID(void) const = 0;
    virtual void setID(int _id) = 0;
    virtual void train(std::vector <Frame*> frames, std::map <std::string, float> params) = 0;
    virtual std::map <uint32_t, std::vector <LimbLabel> > detect(Frame *frame, std::map <std::string, float> params, std::map <uint32_t, std::vector <LimbLabel>> limbLabels);
    virtual std::map <uint32_t, std::vector <LimbLabel>> merge(std::map <uint32_t, std::vector <LimbLabel>> first, std::map <uint32_t, std::vector <LimbLabel>> second, std::map <uint32_t, std::vector <LimbLabel>> secondUnfiltered);
  protected:
    std::vector <Frame*> frames;
    uint32_t maxFrameHeight;
    uint8_t debugLevelParam = 0;
    virtual Frame *getFrame(uint32_t frameId);
    virtual float getBoneLength(cv::Point2f begin, cv::Point2f end);
    virtual float getBoneWidth(float length, BodyPart bodyPart);
    virtual POSERECT <cv::Point2f> getBodyPartRect(BodyPart bodyPart, cv::Point2f j0, cv::Point2f j1, cv::Size blockSize = cv::Size(0, 0));
    virtual cv::Mat rotateImageToDefault(cv::Mat imgSource, POSERECT <cv::Point2f> &initialRect, float angle, cv::Size size);
    virtual LimbLabel generateLabel(BodyPart bodyPart, cv::Point2f j0, cv::Point2f j1, std::string detectorName, float _usedet);
    virtual LimbLabel generateLabel(BodyPart bodyPart, Frame *workFrame, cv::Point2f p0, cv::Point2f p1) = 0;
    virtual LimbLabel generateLabel(float boneLength, float rotationAngle, float x, float y, BodyPart bodyPart, Frame *workFrame);
    virtual float compare(void) = 0;
    virtual std::vector <LimbLabel> filterLimbLabels(std::vector <LimbLabel> &sortedLabels, float uniqueLocationCandidates, float uniqueAngleCandidates);
  };
}
#endif  // _LIBPOSE_DETECTOR_HPP_
