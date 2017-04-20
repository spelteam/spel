#include <iostream>
#include <SURFDetector2.hpp>
#include "projectRunner.hpp"

using namespace std;
using namespace SPEL;

class SURFDetector2Test : public ProjectRunner, public SURFDetector2
{
public:
  SURFDetector2Test(void) : ProjectRunner("SURFDetector2"), SURFDetector2() { };
  void train(vector <Frame*> _frames, map <string, float> params);
  map <uint32_t, vector <LimbLabel> > detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels);
  void DrawSpecific(string outFolder);
  bool drawFramesKeyPoints(string outFolder, Scalar color);
  Mat drawKeyPoints(Frame *frame, std::vector<cv::KeyPoint> keypoints, Scalar color);

private:
  std::map<uint32_t, std::map<uint32_t, std::vector<cv::KeyPoint>>> detected;
};