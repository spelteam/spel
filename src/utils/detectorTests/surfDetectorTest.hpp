#include <iostream>
#include <surfDetector.hpp>
#include "projectRunner.hpp"

using namespace std;

class SURFDetectorTest : public ProjectRunner
{
public:
  SURFDetectorTest(void) : ProjectRunner("surfDetector") { };
  void train(vector <Frame*> _frames, map <string, float> params)
  {
    detector.train(_frames, params);
  }
  vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
  {
    return detector.detect(frame, params, limbLabels);
  }
private:
  SurfDetector detector;
};
