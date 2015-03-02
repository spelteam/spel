#ifndef _LIBPOSE_COLORHISTDETECTOR_HPP_
#define _LIBPOSE_COLORHISTDETECTOR_HPP_

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG
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
    vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels);
    uint8_t getNBins(void);
    vector <Frame*> getFrames() const;
    ColorHistDetector &operator=(const ColorHistDetector &c);
  private:
#ifdef DEBUG
    FRIEND_TEST(colorHistDetectorTest, PrivateFields);
#endif  // DEBUG
    struct PartModel
    {
      PartModel(uint8_t _nBins = 8);
      uint8_t nBins;
      vector <vector <vector <float>>> partHistogramm;
      vector <vector <vector <float>>> bgHistogramm;
      uint32_t sizeFG;
      uint32_t sizeBG;
      uint32_t fgNumSamples;
      uint32_t bgNumSamples;
      vector <uint32_t> fgSampleSizes;
      vector <uint32_t> bgSampleSizes;
      vector <uint32_t> fgBlankSizes;
      PartModel &operator=(PartModel &model);
    };
    int id;
    const uint8_t nBins;
    map <int32_t, PartModel> partModels;
    //vector <Frame*> frames;
    float computePixelBelongingLikelihood(const PartModel &partModel, uint8_t r, uint8_t g, uint8_t b);
    void setPartHistogramm(PartModel &partModel, const vector <Point3i> &partColors);
    void addPartHistogramm(PartModel &partModel, const vector <Point3i> &partColors, uint32_t nBlankPixels);
    void addBackgroundHistogramm(PartModel &partModel, const vector <Point3i> &bgColors);
    float getAvgSampleSizeFg(const PartModel &partModel);
    float getAvgSampleSizeFgBetween(const PartModel &partModel, uint32_t s1, uint32_t s2);
    float matchPartHistogramsED(const PartModel &partModelPrev, const PartModel &partModel);
    map <int32_t, Mat> buildPixelDistributions(Frame *frame);
    map <int32_t, Mat> buildPixelLabels(Frame *frame, map <int32_t, Mat> pixelDistributions);
    LimbLabel generateLabel(BodyPart bodyPart, Frame *frame, map <int32_t, Mat> pixelDistributions, map <int32_t, Mat> pixelLabels, Point2f j0, Point2f j1);
};

#endif  // _LIBPOSE_COLORHISTDETECTOR_HPP_

