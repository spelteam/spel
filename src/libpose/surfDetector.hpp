#ifndef _LIBPOSE_SURFDETECTOR_HPP_
#define _LIBPOSE_SURFDETECTOR_HPP_

#include "detector.hpp"

class SurfDetector : public Detector
{
  public:
    void train(vector <Frame> frames, int id);
    vector <LimbLabel> detect(Frame frame, vector <float> params);
  private:
};

#endif  // _LIBPOSE_SURFDETECTOR_HPP_
