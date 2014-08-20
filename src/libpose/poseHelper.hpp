#ifndef _POSEHELPER_HPP_
#define _POSEHELPER_HPP_

#include <opencv2/opencv.hpp>

using namespace cv;

// class with common functions
class PoseHelper
{
  public:
    template <typename T> static double distSquared(T one, T two)
    {
      return pow(one.x - two.x, 2.0) + pow(one.y - two.y, 2.0);
    }
    static double angle2D(double x1, double y1, double x2, double y2);
    template <typename T> static T rotatePoint2D(const T point, const T pivot, const float degrees)
    {
      double radians = degrees * M_PI / 180.0;
      Point2f pt, cnt;
      pt = point;
      cnt = pivot;
      pt = pt - cnt;
      Point2f result;
      result.x = pt.x * cosf(radians) - pt.y * sinf(radians);
      result.y = pt.x * sinf(radians) + pt.y * cosf(radians);
      result = result + cnt;
      T res = result;
      return res;
    }
};

#endif  // _POSEHELPER_HPP_

