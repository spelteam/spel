#ifndef _LIBPOSE_SURFDETECTOR_HPP_
#define _LIBPOSE_SURFDETECTOR_HPP_

#include "detector.hpp"

class SurfDetector : public Detector
{
  public:
    int getID(void);
    void setID(int _id);
    void train(vector <Frame*> frames, map <string, float>);
    vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params);
  private:
    int id;
};

#endif  // _LIBPOSE_SURFDETECTOR_HPP_

