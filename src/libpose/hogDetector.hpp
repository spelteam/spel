#ifndef _LIBPOSE_HOGDETECTOR_HPP_
#define _LIBPOSE_HOGDETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include "detector.hpp"

using namespace std;
using namespace cv;

class HogDetector : public Detector
{
  public:
    int getID(void);
    void setID(int _id);
    void train(vector <Frame*> _frames, map <string, float> params);
    vector <vector <LimbLabel>> detect(Frame *frame, map <string, float> params);
  private:
    struct PartModel
    {
      //map <PHPoint<uint32_t>, vector <float>> partDescriptors;
      POSERECT <Point2f> partModelRect;
      vector <float> descriptors;
      float compare(PartModel ethalon, uint8_t nBins);
    };
    int id;
    //map <uint32_t, map <PHPoint<uint32_t>, vector <float>>> rawDescriptors;
    //map <uint32_t, map <uint32_t, map<PHPoint<float>, vector <float>>>> frameBodyPartDescriptors;

    //map <uint32_t, map <uint32_t, PartModel>> rawPartModelDescriptors;
    //map <uint32_t, PartModel> partModelAverageDescriptors;

    map <uint32_t, Size> partSize;
    map <uint32_t, map <uint32_t, PartModel>> partModels;

    //map <PHPoint<uint32_t>, vector <float>> computeDescriptors(HOGDescriptor detector, Size wndSize, Size wndStride, Size blockSize, Size blockStride, Size cellSize, int nbins, Frame *frame);
    //void parseBodyPartDescriptors(Frame *frame, map <PHPoint<uint32_t>, vector <float>> currentFrameRawDescriptors);
    LimbLabel generateLabel(BodyPart bodyPart, Frame *frame, Point2f j0, Point2f j1, map <PHPoint<uint32_t>, vector <float>> deswcriptors, uint8_t nBins);

    map <uint32_t, Size> getMaxBodyPartHeightWidth(vector <Frame*> frames, Size blockSize);
    map <uint32_t, PartModel> computeDescriptors(Frame *frame, int nbins, Size wndSize, Size wndStride, Size blockSize, Size blockStride, Size cellSize, double wndSigma, double thresholdL2hys, bool gammaCorrection, int nlevels);
};

#endif  // _LIBPOSE_HOGDETECTOR_HPP_
