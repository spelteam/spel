#ifndef _LIBPOSE_DETECTOR_HPP_
#define _LIBPOSE_DETECTOR_HPP_

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
};

#endif  // _LIBPOSE_DETECTOR_HPP_

