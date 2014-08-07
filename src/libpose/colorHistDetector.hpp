#ifndef _LIBPOSE_COLORHISTDETECTOR_HPP_
#define _LIBPOSE_COLORHISTDETECTOR_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include "detector.hpp"

using namespace std;
using namespace cv;

class ColorHistDetector : public Detector
{
  public:
    ColorHistDetector(uint8_t _nBins = 8);  // default is 8 for 32 bit colourspace
    int getID(void);
    void setID(int _id);
    void train(vector <Frame*> frames, int id);
    vector <vector <LimbLabel> > detect(Frame *frame, vector <float> params);
    uint8_t getNBins(void);
  private:
    struct PartModel
    {
      PartModel(uint8_t _nBins);
      const uint8_t nBins;
      vector <vector <vector <float>>> partHistogramm;
      uint32_t sizeFG;
      bool uniqueExists;
      uint32_t fgNumSamples;
      vector <uint32_t> fgSampleSizes;
      vector <uint32_t> fgBlankSizes;
    };
    int id;
    const uint8_t nBins;
    vector <PartModel> partModels;
    float computePixelBelongingLikelihood(uint32_t nPartModel, uint8_t r, uint8_t g, uint8_t b);
    void setPartHistogramm(uint32_t nPartModel, const vector <Point3i> &partColors);
    void addPartHistogramm(uint32_t nPartModel, const vector <Point3i> &partColors, uint32_t nBlankPixels);
    float getAvgSampleSizeFg(uint32_t nPartModel);
    float getAvgSampleSizeFgBetween(uint32_t nPartModel, uint32_t s1, uint32_t s2);
};

#endif  // _LIBPOSE_COLORHISTDETECTOR_HPP_

