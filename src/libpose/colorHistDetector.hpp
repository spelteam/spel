#ifndef _LIBPOSE_COLORHISTDETECTOR_HPP_
#define _LIBPOSE_COLORHISTDETECTOR_HPP_

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
    void train(vector <Frame*> _frames, map <string, float> params);
    vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params);
    uint8_t getNBins(void);
  private:
    struct PartModel
    {
      PartModel(uint8_t _nBins);
      const uint8_t nBins;
      vector <vector <vector <float>>> partHistogramm;
      vector <vector <vector <float>>> bgHistogramm;
      uint32_t sizeFG;
      uint32_t sizeBG;
      bool uniqueExists;
      uint32_t fgNumSamples;
      uint32_t bgNumSamples;
      vector <uint32_t> fgSampleSizes;
      vector <uint32_t> bgSampleSizes;
      vector <uint32_t> fgBlankSizes;
    };
    int id;
    const uint8_t nBins;
    map <int32_t, PartModel> partModels;
    vector <Mat> _pixelDistributions;
    float computePixelBelongingLikelihood(const PartModel &partModel, uint8_t r, uint8_t g, uint8_t b);
    void setPartHistogramm(PartModel &partModel, const vector <Point3i> &partColors);
    void addPartHistogramm(PartModel &partModel, const vector <Point3i> &partColors, uint32_t nBlankPixels);
    void addBackgroundHistogramm(PartModel &partModel, const vector <Point3i> &bgColors);
    float getAvgSampleSizeFg(const PartModel &partModel);
    float getAvgSampleSizeFgBetween(const PartModel &partModel, uint32_t s1, uint32_t s2);
    float matchPartHistogramsED(const PartModel &partModelPrev, const PartModel &partModel, uint32_t prevSizeIndex, uint32_t nextSizeIndex);
    vector <Mat> buildPixelDistributions(Frame *frame);
    vector <Mat> buildPixelLabels(Frame *frame);
    LimbLabel generateLabel(BodyPart bodyPart, Frame *frame, vector <Mat> pixelDistributions, vector <Mat> pixelLabels);
//TODO (Vitaliy Koshura)
// Seems to be common functions. Need to move it somewhere
    template <typename T> double distSquared(T one, T two) { return pow(one.x - two.x, 2.0) + pow(one.y - two.y, 2.0); }
    double angle2D(double x1, double y1, double x2, double y2);
    template <typename T> T rotatePoint2D(const T point, const T pivot, const float degrees)
    {
      double radians = degrees * M_PI / 180.0;
      Point2f pt, cnt;
      pt = point;
      cnt = pivot;
      pt = pt - cnt;
      Point2f result;
      result.x = pt.x * cosf(radians) - pt.y * sinf(radians);
      result.y = pt.x * sinf(radians) + pt.y * cosf(radians);
      result = result + cnt;
      T res = result;
      return res;
    }
};

#endif  // _LIBPOSE_COLORHISTDETECTOR_HPP_

