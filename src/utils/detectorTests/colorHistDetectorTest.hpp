#include <iostream>
#include <colorHistDetector.hpp>
#include "projectRunner.hpp"

using namespace std;
using namespace SPEL;

class ColorHistDetectorTest : public ProjectRunner, public ColorHistDetector
{
public:
  ColorHistDetectorTest(void) : ProjectRunner("colorHistDetector"), ColorHistDetector() { };
  void train(vector <Frame*> _frames, map <string, float> params);
  vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels);
};
