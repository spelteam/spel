#include "colorHistDetectorTest.hpp"

void ColorHistDetectorTest::train(vector <Frame*> _frames, map <string, float> params)
{
  ColorHistDetector::train(_frames, params);
}
map <uint32_t, vector <LimbLabel> > ColorHistDetectorTest::detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels)
{
  return ColorHistDetector::detect(frame, params, limbLabels);
}
