#ifndef _LIBPOSE_COLORHISTDETECTOR_HPP_
#define _LIBPOSE_COLORHISTDETECTOR_HPP_

#include "detector.hpp"

class ColorHistDetector : public Detector
{
  public:
    int getID(void);
    void setID(int _id);
    void train(vector <Frame> frames, int id);
    vector <vector <LimbLabel> > detect(Frame *frame, vector <float> params);
  private:
    int id;
};

#endif  // _LIBPOSE_COLORHISTDETECTOR_HPP_

