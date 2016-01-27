#ifndef _POSEHELPER_HPP_
#define _POSEHELPER_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif  // WINDOWS

// OpenCV
#include <opencv2/opencv.hpp>
#if OpenCV_VERSION_MAJOR >= 3
#include <opencv2/core/types_c.h>
#endif

// tree.hh
#include <tree.hh>

#include "limbLabel.hpp"

namespace SPEL
{
  /// class with common functions
  class spelHelper
  {
  public:
    ///find squared distance between two points
    ///Arguments:
    ///one - first point
    ///two - second point
    ///Result:
    ///squared distance between one and two
    template <typename T>
    static double distSquared(T one, T two)
    {
      return pow(one.x - two.x, 2.0) + pow(one.y - two.y, 2.0);
    }

    template <typename T>
    static double distSquared3d(T one, T two)
    {
      return pow(one.x - two.x, 2.0) + pow(one.y - two.y, 2.0) + pow(one.z - two.z, 2.0);
    }

    ///find angle between two vectors on a plane
    ///Arguments:
    ///x1, y1 - first vector
    ///x2, y2 - second vector
    ///Result:
    ///angle between first and second vector,
    ///positive counter clockwise
    ///angle is in the range [-PI; PI]
    template <typename T>
    static T angle2D(T x1, T y1, T x2, T y2)
    {
      const T zero = static_cast<T> (0.0);
      //input has a zero vector
      //zero vector is both parallel and perpendicular to every vector
      if ((x1 == zero && y1 == zero) || (x2 == zero && y2 == zero))
        return zero;

      //angle between Ox and first vector
      auto theta1 = atan2(y1, x1);
      //angle between Ox and second vector
      auto theta2 = atan2(y2, x2);
      //angle between first and second vector
      auto dtheta = theta2 - theta1;
      //normalize angle to range [-PI;PI]
      while (dtheta > M_PI)
        dtheta -= (M_PI * static_cast<T> (2.0));
      while (dtheta < -M_PI)
        dtheta += (M_PI * static_cast<T> (2.0));

      return dtheta;
    }

    template <typename T, typename D, typename E>
    static T interpolateFloat(T prevAngle, T nextAngle, D step, E numSteps) noexcept
    {
      const T zero = static_cast<T> (0.0);
      T t;
      if (numSteps != 0)
        t = step / static_cast<T>(numSteps);
      else
        t = zero;
      return prevAngle * (static_cast<T> (1.0) - t) + nextAngle * t;
    }
    //
    template <typename T, typename D>
    static T rotatePoint2D(const T point, const T pivot, const D degrees) noexcept
    {
      auto radians = degrees * M_PI / static_cast<D> (180.0);
      auto cnt = pivot;
      auto pt = point - cnt;
      T result;
      result.x = pt.x * cosf(radians) - pt.y * sinf(radians);
      result.y = pt.x * sinf(radians) + pt.y * cosf(radians);
      result = result + cnt;
      return result;
    }
    ///copies tree
    ///Arguments:
    ///dst - copy to
    ///src - copy from
    ///Result:
    ///dst is copy of src
    template <class T, class tree_node_allocator>
    static void copyTree(tree <T, tree_node_allocator> &dst, const tree <T, tree_node_allocator> &src)
    {
      dst.clear();
      auto it = src.begin(), to = dst.begin();
      while (it != src.end())
      {
        to = dst.insert(to, (*it));
        it.skip_children();
        ++it;
      }
      to = dst.begin();
      it = src.begin();
      while (it != src.end())
      {
        to = dst.replace(to, it);
        to.skip_children();
        it.skip_children();
        ++to;
        ++it;
      }
    }

    static void RecalculateScoreIsWeak(std::vector <LimbLabel> &labels, std::string detectorName, float standardDiviationTreshold);
  };

  ///represents rectangle
  template <class T>
  class POSERECT
  {
  public:
    POSERECT(void)
      : point1(),
      point2(),
      point3(),
      point4() {}
    POSERECT(const POSERECT<T>& poserect)
      : point1(poserect.point1),
      point2(poserect.point2),
      point3(poserect.point3),
      point4(poserect.point4) {}
    POSERECT(POSERECT<T>&& poserect)
      : point1(std::move(poserect.point1)),
      point2(std::move(poserect.point2)),
      point3(std::move(poserect.point3)),
      point4(std::move(poserect.point4)) {}
    POSERECT(T _point1, T _point2, T _point3, T _point4)
      : point1(_point1),
      point2(_point2),
      point3(_point3),
      point4(_point4) {}
    T point1;
    T point2;
    T point3;
    T point4;
    ///check whether point lies in rectangle
    ///Arguments:
    ///point - point which checks
    ///Result:
    ///1 - point lies in rectangle
    ///-1 - point doesn't lies in rectangle
    ///0 - point lies on the edge(vertex)
    int8_t containsPoint(const T &point) const
    {
      std::vector <T> contour;
      contour.push_back(point1);
      contour.push_back(point2);
      contour.push_back(point3);
      contour.push_back(point4);

      return (int8_t)pointPolygonTest(contour, point, false);
    }
    ///convert rectangle to vector of points
    std::vector <T> asVector(void)
    {
      std::vector <T> contour;
      contour.push_back(point1);
      contour.push_back(point2);
      contour.push_back(point3);
      contour.push_back(point4);
      return contour;
    }

    template <typename D>
    void GetMinMaxXY(D &minx, D &miny, D &maxx, D &maxy)
    {
      minx = std::min(std::min(point1.x, point2.x), std::min(point3.x, point4.x));
      maxx = std::max(std::max(point1.x, point2.x), std::max(point3.x, point4.x));
      miny = std::min(std::min(point1.y, point2.y), std::min(point3.y, point4.y));
      maxy = std::max(std::max(point1.y, point2.y), std::max(point3.y, point4.y));
    }

    template <typename D>
    D GetCenter(void) const
    {
      D center1 = 0.5 * point1 + 0.5 * point3;
      D center2 = 0.5 * point2 + 0.5 * point4;
      float dist = (float)sqrt(spelHelper::distSquared(center1, center2));
      if (dist > 0.001)
      {
        throw std::logic_error("Rect center couldn't be found");
      }
      else
      {
        return center1;
      }
    }

    POSERECT<T>& operator=(const POSERECT<T>& rect) {
      if (&rect == this) return *this;

      point1 = rect.point1;
      point2 = rect.point2;
      point3 = rect.point3;
      point4 = rect.point4;

      return *this;
    }

    POSERECT<T>& operator=(POSERECT<T>&& rect) {
      std::swap(point1, rect.point1);
      std::swap(point2, rect.point2);
      std::swap(point3, rect.point3);
      std::swap(point4, rect.point4);

      return *this;
    }

    bool operator==(const POSERECT <T> &rect) const
    {
      return (this->point1 == rect.point1 && this->point2 == rect.point2 && this->point3 == rect.point3 && this->point4 == rect.point4);
    }

    bool operator!=(const POSERECT <T> &rect) const
    {
      return !(*this == rect);
    }

    template <typename D>
    D RectSize(void)
    {
      return (D(sqrt(spelHelper::distSquared(point2, point3)), sqrt(spelHelper::distSquared(point1, point2))));
    }

  };

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

#endif  // _POSEHELPER_HPP_
