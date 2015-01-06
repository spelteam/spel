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
    void train(vector <Frame*> frames, map <string, float> params);
    vector <vector <LimbLabel>> detect(Frame *frame, map <string, float> params);
  private:
    int id;
    //map <uint32_t, vector <float>> frameHOGDescriptors;
    map <uint32_t, map <PHPoint<uint32_t>, vector <float>>> rawDescriptors;
    map <uint32_t, map <uint32_t, map<PHPoint<float>, vector <float>>>> frameBodyPartDescriptors;

    map <PHPoint<uint32_t>, vector <float>> computeDescriptors(HOGDescriptor detector, Size wndSize, Size wndStride, Size blockSize, Size blockStride, Size cellSize, int nbins, Frame *frame);
    void parseBodyPartDescriptors(Frame *frame, map <PHPoint<uint32_t>, vector <float>> currentFrameRawDescriptors);
      
};

#endif  // _LIBPOSE_HOGDETECTOR_HPP_

