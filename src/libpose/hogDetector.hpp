#ifndef _LIBPOSE_HOGDETECTOR_HPP_
#define _LIBPOSE_HOGDETECTOR_HPP_

#include "detector.hpp"

class HogDetector : public Detector
{
  public:
    int getID(void);
    void setID(int _id);
    void train(vector <Frame*> frames);
    vector <vector <LimbLabel> > detect(Frame *frame, vector <float> params);
  private:
    int id;
};

#endif  // _LIBPOSE_HOGDETECTOR_HPP_

