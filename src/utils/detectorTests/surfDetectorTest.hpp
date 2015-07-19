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
  vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels);
  void DrawSpecific(string outFolder);
  bool drawSURFKeyPoints(map <uint32_t, map <uint32_t, PartModel>> partModels, map <uint32_t, map <uint32_t, vector <PartModel>>> labelModels, string outFolder, Scalar color);
  Mat drawSURFKeyPoints(Frame *frame, PartModel model, Scalar color);
private:
  SurfDetector detector;
};
