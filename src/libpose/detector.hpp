#ifndef _LIBPOSE_DETECTOR_HPP_
#define _LIBPOSE_DETECTOR_HPP_

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
#include "poseHelper.hpp"
#include "sequence.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"

class Detector
{
  public:
    virtual int getID(void) = 0;
    virtual void setID(int _id) = 0;
    virtual void train(vector <Frame*> frames, map <string, float> params) = 0;
    virtual vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels) = 0;
    virtual vector <vector <LimbLabel>> merge(vector <vector <LimbLabel>> first, vector <vector <LimbLabel>> second);
  protected:
    vector <Frame*> frames;
    uint32_t maxFrameHeight;
    uint8_t debugLevelParam = 0;
    virtual float getBoneLength(Point2f begin, Point2f end);
    virtual float getBoneWidth(float length, BodyPart bodyPart);
    virtual POSERECT <Point2f> getBodyPartRect(BodyPart bodyPart, Point2f j0, Point2f j1, Size blockSize = Size(0, 0));
    virtual Mat rotateImageToDefault(Mat imgSource, POSERECT <Point2f> &initialRect, float angle, Size size);
    virtual LimbLabel generateLabel(BodyPart bodyPart, Point2f j0, Point2f j1, string detectorName, float _usedet);
    virtual float compare(void) = 0;
};

#endif  // _LIBPOSE_DETECTOR_HPP_

