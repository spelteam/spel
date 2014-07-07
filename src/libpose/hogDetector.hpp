#ifndef _LIBPOSE_HOGDETECTOR_HPP_
#define _LIBPOSE_HOGDETECTOR_HPP_

#include "detector.hpp"

class HogDetector : public Detector
{
  public:
    void train(vector <Frame> frames, int id);
    vector <LimbLabel> detect(Frame frame, vector <float> params);
  private:
};

#endif  // _LIBPOSE_HOGDETECTOR_HPP_
