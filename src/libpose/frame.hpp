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
    virtual vector <Point2f> getPartPolygon(int partID);
  private:
    int id;
    Mat image;
    Mat mask;
    Skeleton skeleton;
    Point2f groundPoint;
};

#endif  // _LIBPOSE_FRAME_HPP_

