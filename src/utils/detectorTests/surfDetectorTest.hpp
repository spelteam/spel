#include <iostream>
#include <surfDetector.hpp>
#include "projectRunner.hpp"

using namespace std;
using namespace SPEL;

class SURFDetectorTest : public ProjectRunner, public SurfDetector
{
public:
  SURFDetectorTest(void) : ProjectRunner("surfDetector"), SurfDetector() { };
  void train(vector <Frame*> _frames, map <string, float> params);
  map <uint32_t, vector <LimbLabel> > detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels);
  void DrawSpecific(string outFolder);
  bool drawSURFKeyPoints(map <uint32_t, map <uint32_t, PartModel>> partModels, string outFolder, Scalar color);
  Mat drawSURFKeyPoints(Frame *frame, PartModel model, Scalar color);
};
