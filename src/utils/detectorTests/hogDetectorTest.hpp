#include <iostream>
#include <hogDetector.hpp>
#include "projectRunner.hpp"

using namespace std;

class HoGDetectorTest : public ProjectRunner
{
public:
  HoGDetectorTest(void) : ProjectRunner("hogDetector") { };
  void train(vector <Frame*> _frames, map <string, float> params);
  vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels);
  void DrawSpecific(string outFolder);
  bool drawHoGDescriptors(map <uint32_t, map <uint32_t, HogDetector::PartModel>> partModels, map <uint32_t, map <uint32_t, vector <HogDetector::PartModel>>> labelModels, string outFolder, Scalar lineColor, Scalar descriptorColor, int lineWidth, int descriptorWidth, Size cellSize, uint8_t nbins);
  Mat drawHoGDescriptors(HogDetector::PartModel model, Scalar lineColor, Scalar descriptorColor, int lineWidth, int descriptorWidth, Size cellSize, uint8_t nbins);

private:
  HogDetector detector;
};
