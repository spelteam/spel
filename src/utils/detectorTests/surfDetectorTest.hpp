#include <iostream>
#include <surfDetector.hpp>
#include "projectRunner.hpp"

using namespace std;

class SURFDetectorTest : public ProjectRunner
{
public:
  SURFDetectorTest(void) : ProjectRunner("surfDetector") { };
  void train(vector <Frame*> _frames, map <string, float> params);
  vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels);
  void DrawSpecific(string outFolder);
private:
  SurfDetector detector;
};
