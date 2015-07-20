#include "colorHistDetectorTest.hpp"

void ColorHistDetectorTest::train(vector <Frame*> _frames, map <string, float> params)
{
  ColorHistDetector::train(_frames, params);
}
vector <vector <LimbLabel> > ColorHistDetectorTest::detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
{
  return ColorHistDetector::detect(frame, params, limbLabels);
}
