#include "surfDetectorTest.hpp"

void SURFDetectorTest::train(vector <Frame*> _frames, map <string, float> params)
{
  detector.train(_frames, params);
}
vector <vector <LimbLabel> > SURFDetectorTest::detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
{
  return detector.detect(frame, params, limbLabels);
}

void SURFDetectorTest::DrawSpecific(string outFolder)
{

}
