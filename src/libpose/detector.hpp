#ifndef _LIBPOSE_DETECTOR_HPP_
#define _LIBPOSE_DETECTOR_HPP_

#include <vector>
#include "frame.hpp"
#include "limbLabel.hpp"

class Detector
{
  public:
    virtual void train(vector <Frame> frames, int id) = 0;
    virtual vector <LimbLabel> detect(Frame frame, vector <float> params) = 0;
  private:
    const int id;
    // T model;
};

#endif  // _LIBPOSE_DETECTOR_HPP_
