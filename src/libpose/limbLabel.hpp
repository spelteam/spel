#ifndef _LIBPOSE_LIMBLABEL_HPP_
#define _LIBPOSE_LIMBLABEL_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

class LimbLabel
{
  public:
  private:
    int limbID;
    Point2f center;
    float angle;
    vector <Point2f> polygon;    
};

#endif  // _LIBPOSE_LIMBLABEL_HPP_
