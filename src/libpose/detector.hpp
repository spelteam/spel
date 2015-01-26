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
#include "frame.hpp"
#include "limbLabel.hpp"
#include "poseHelper.hpp"

class Detector
{
  public:
    virtual int getID(void) = 0;
    virtual void setID(int _id) = 0;
    virtual void train(vector <Frame*> frames, map <string, float> params) = 0;
    virtual vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params) = 0;
  private:
    // T model;    
  protected:
    vector <Frame*> frames;
    virtual void getNeighborFrame(Frame **curFrame, Frame **prevFrame, Frame **nextFrame, uint32_t &step, uint32_t &stepCount);
};

#endif  // _LIBPOSE_DETECTOR_HPP_

