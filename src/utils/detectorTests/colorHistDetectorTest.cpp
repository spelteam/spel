#include "colorHistDetectorTest.hpp"

void ColorHistDetectorTest::train(vector <Frame*> _frames, map <string, float> params)
{
  detector.train(_frames, params);
}
vector <vector <LimbLabel> > ColorHistDetectorTest::detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
{
  return detector.detect(frame, params, limbLabels);
}
