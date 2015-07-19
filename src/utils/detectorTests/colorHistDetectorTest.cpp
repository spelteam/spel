#include "colorHistDetectorTest.hpp"

void ColorHistDetectorTest::train(vector <Frame*> _frames, map <string, float> params)
{
  train(_frames, params);
}
vector <vector <LimbLabel> > ColorHistDetectorTest::detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
{
  return detect(frame, params, limbLabels);
}
