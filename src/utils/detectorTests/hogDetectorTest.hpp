#include <iostream>
#include <hogDetector.hpp>
#include "projectRunner.hpp"

using namespace std;

class HoGDetectorTest : public ProjectRunner, public HogDetector
{
public:
  HoGDetectorTest(void) : ProjectRunner("hogDetector"), HogDetector() { };
  void train(vector <Frame*> _frames, map <string, float> params);
  vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels);
  void DrawSpecific(string outFolder);
  bool drawHoGDescriptors(map <uint32_t, map <uint32_t, PartModel>> partModels, map <uint32_t, map <uint32_t, vector <PartModel>>> labelModels, string outFolder, Scalar lineColor, Scalar descriptorColor, int lineWidth, int descriptorWidth, Size cellSize, uint8_t nbins);
  Mat drawHoGDescriptors(PartModel model, Scalar lineColor, Scalar descriptorColor, int lineWidth, int descriptorWidth, Size cellSize, uint8_t nbins);
};
