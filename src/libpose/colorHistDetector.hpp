#ifndef _LIBPOSE_COLORHISTDETECTOR_HPP_
#define _LIBPOSE_COLORHISTDETECTOR_HPP_

#include <vector>
#include "detector.hpp"

using namespace std;

class ColorHistDetector : public Detector
{
  public:
    ColorHistDetector(void);
    int getID(void);
    void setID(int _id);
    void train(vector <Frame> frames, int id);
    vector <vector <LimbLabel> > detect(Frame *frame, vector <float> params);
    uint8_t getNBins(void);
    float computePixelBelongingLikelihood(uint8_t r, uint8_t g, uint8_t b);
  private:
    int id;
    const uint8_t nBins;
    vector <vector <vector <float>>> partHistogramm;
};

#endif  // _LIBPOSE_COLORHISTDETECTOR_HPP_

