#ifndef _LIBPOSE_FRAME_HPP_
#define _LIBPOSE_FRAME_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include "skeleton.hpp"

using namespace std;
using namespace cv;

class Frame
{
  public:
    vector <Point2f> getPartPolygon(int partID);
  private:
    Mat image;
    Mat mask;
    Skeleton skeleton;
};

#endif  // _LIBPOSE_FRAME_HPP_
