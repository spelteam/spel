#include <iostream>
#include <hogDetector.hpp>
#include "projectRunner.hpp"

using namespace std;

class HoGDetectorTest : public ProjectRunner
{
public:
  HoGDetectorTest(void) : ProjectRunner("hogDetector") { };
  void train(vector <Frame*> _frames, map <string, float> params)
  {
    detector.train(_frames, params);
  }
  vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
  {
    return detector.detect(frame, params, limbLabels);
  }
private:
  HogDetector detector;
};
