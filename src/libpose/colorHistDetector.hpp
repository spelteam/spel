#ifndef _LIBPOSE_COLORHISTDETECTOR_HPP_
#define _LIBPOSE_COLORHISTDETECTOR_HPP_

#include "detector.hpp"

class ColorHistDetector : public Detector
{
  public:
    void train(vector <Frame> frames, int id);
    vector <LimbLabel> detect(Frame frame, vector <float> params);
  private:
};

#endif  // _LIBPOSE_COLORHISTDETECTOR_HPP_

