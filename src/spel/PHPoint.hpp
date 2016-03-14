#ifndef PHPOINT_HPP_
#define PHPOINT_HPP_

// OpenCV
#include <opencv2/opencv.hpp>
#if OpenCV_VERSION_MAJOR >= 3
#include <opencv2/core/types_c.h>
#endif

namespace SPEL
{
  template <typename T>
  class PHPoint : public cv::Point_ < T >
  {
  public:
    /// various constructors
    PHPoint() : cv::Point_<T>() {}
    PHPoint(T _x, T _y) : cv::Point_<T>(_x, _y) {}
    PHPoint(const cv::Point_<T>& pt) : cv::Point_<T>(pt) {}
    PHPoint(const CvPoint& pt) : cv::Point_<T>(pt) {}
    PHPoint(const CvPoint2D32f& pt) : cv::Point_<T>(pt) {}
    PHPoint(const cv::Size_<T>& sz) : cv::Point_<T>(sz) {}
    PHPoint(const cv::Vec<T, 2>& v) : cv::Point_<T>(v) {}

    bool operator < (const PHPoint& pt) const
    {
      return (this->x < pt.x || (this->x == pt.x && this->y < pt.y));
    }
  };

  template <typename T>
  class PHPoint3 : public cv::Point3_ < T >
  {
  public:
    /// various constructors
    PHPoint3() : cv::Point3_<T>() {}
    PHPoint3(T _x, T _y, T _z) : cv::Point3_<T>(_x, _y, _z) {}
    PHPoint3(const cv::Point3_<T>& pt) : cv::Point3_<T>(pt) {}
    explicit PHPoint3(const cv::Point_<T>& pt) : cv::Point3_<T>(pt) {}
    PHPoint3(const CvPoint3D32f& pt) : cv::Point3_<T>(pt) {}
    PHPoint3(const cv::Vec<T, 3>& v) : cv::Point3_<T>(v) {}

    bool operator < (const PHPoint3& pt) const
    {
      return (this->x < pt.x || (this->x == pt.x && this->y < pt.y) || (this->x == pt.x && this->y == pt.y && this->z < pt.z));
    }
  };
}

#endif // PHPOINT_HPP_
