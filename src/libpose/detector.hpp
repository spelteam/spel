#ifndef _LIBPOSE_DETECTOR_HPP_
#define _LIBPOSE_DETECTOR_HPP_

#include <vector>
#include "frame.hpp"
#include "limbLabel.hpp"

class Detector
{
  public:
    virtual int getID(void) = 0;
    virtual void setID(int _id) = 0;
    virtual void train(vector <Frame*> frames, int id) = 0;
    virtual vector <vector <LimbLabel> > detect(Frame *frame, vector <float> params) = 0;
  private:
    // T model;
};

#endif  // _LIBPOSE_DETECTOR_HPP_

