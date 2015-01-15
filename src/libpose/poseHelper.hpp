#ifndef _POSEHELPER_HPP_
#define _POSEHELPER_HPP_

#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif  // WINDOWS

#include <opencv2/opencv.hpp>
#include <tree.hh>

using namespace std;
using namespace cv;

template <class T>
struct TPOSERECT
{
  T p1;
  T p2;
  T p3;
  T p4;
};
//represents rectangle
template <class T>
struct POSERECT
{
  POSERECT() : point1(), point2(), point3(), point4() {}
  POSERECT(T _point1, T _point2, T _point3, T _point4) : point1(_point1), point2(_point2), point3(_point3), point4(_point4) {}
  T point1;
  T point2;
  T point3;
  T point4;
  //check whether point lies in rectangle
  //Arguments:
  //point - point which checks
  //Result:
  //1 - point lies in rectangle
  //-1 - point doesn't lies in rectangle
  //0 - point lies on the edge(vertex)
  int8_t containsPoint(T point)
  {
    vector <T> contour;
    contour.push_back(point1);
    contour.push_back(point2);
    contour.push_back(point3);
    contour.push_back(point4);
    return pointPolygonTest(contour, point, false);
  }
  //convert rectangle to vector of points
  vector <T> asVector()
  {
    vector <T> contour;
    contour.push_back(point1);
    contour.push_back(point2);
    contour.push_back(point3);
    contour.push_back(point4);
    return contour;
  }

  template <typename D> void GetMinMaxXY(D &minx, D &miny, D &maxx, D &maxy)
  {
    minx = min(min(point1.x, point2.x), min(point3.x, point4.x));
    maxx = max(max(point1.x, point2.x), max(point3.x, point4.x));
    miny = min(min(point1.y, point2.y), min(point3.y, point4.y));
    maxy = max(max(point1.y, point2.y), max(point3.y, point4.y));
  }

  bool operator==(const POSERECT <T> &rect) const
  {
    return (this->point1 == rect.point1 && this->point2 == rect.point2 && this->point3 == rect.point3 && this->point4 == rect.point4);
  }

  bool operator!=(const POSERECT <T> &rect) const
  {
    return !(*this == rect);
  }
};

template <typename T>
class PHPoint : public Point_<T>
{
public:
  // various constructors
  PHPoint() : Point_<T>() {}
  PHPoint(T _x, T _y) : Point_<T>(_x, _y) {}
  PHPoint(const Point_<T>& pt) : Point_<T>(pt) {}
  PHPoint(const CvPoint& pt) : Point_<T>(pt) {}
  PHPoint(const CvPoint2D32f& pt) : Point_<T>(pt) {}
  PHPoint(const Size_<T>& sz) : Point_<T>(sz) {}
  PHPoint(const Vec<T, 2>& v) : Point_<T>(v) {}

  bool operator < (const PHPoint& pt) const
  {
    return (this->x < pt.x && this->y < pt.y);
  }
};

template <typename T>
class PHPoint3 : public Point3_ < T >
{
public:
  // various constructors
  PHPoint3() : Point3_<T>() {}
  PHPoint3(T _x, T _y, T _z) : Point3_<T>(_x, _y, _z) {}
  PHPoint3(const Point3_<T>& pt) : Point3_<T>(pt) {}
  explicit PHPoint3(const Point_<T>& pt) : Point3_<T>(pt) {}
  PHPoint3(const CvPoint3D32f& pt) : Point3_<T>(pt) {}
  PHPoint3(const Vec<T, 3>& v) : Point3_<T>(v) {}

  bool operator < (const PHPoint3& pt) const
  {
    return (this->x < pt.x && this->y < pt.y && this->z < pt.z);
  }
};

// class with common functions
class PoseHelper
{
  public:
    //find squared distance between two points
    //Arguments:
    //one - first point
    //two - second point
    //Result:
    //squared distance between one and two
    template <typename T> static double distSquared(T one, T two)
    {
      return pow(one.x - two.x, 2.0) + pow(one.y - two.y, 2.0);
    }
    //find angle between two vectors on a plane
    //Arguments:
    //x1, y1 - first vector
    //x2, y2 - second vector
    //Result:
    //angle between first and second vector,
    //positive counter clockwise
    //angle is in the range [-PI; PI]
    static double angle2D(double x1, double y1, double x2, double y2);
    //
    template <typename T> static T rotatePoint2D(const T point, const T pivot, const float degrees)
    {
      double radians = degrees * M_PI / 180.0;
      T pt, cnt;
      pt = point;
      cnt = pivot;
      pt -= cnt;
      T result;
      result.x = pt.x * cosf(radians) - pt.y * sinf(radians);
      result.y = pt.x * sinf(radians) + pt.y * cosf(radians);
      result = result + cnt;
      return result;
    }
    //copies tree
    //Arguments:
    //dst - copy to
    //src - copy from
    //Result:
    //dst is copy of src
    template <class T, class tree_node_allocator>
    static void copyTree(tree <T, tree_node_allocator> &dst, const tree <T, tree_node_allocator> &src) 
    {
      dst.clear();
      auto it = src.begin(), to = dst.begin();
      while(it != src.end())
      {
        to = dst.insert(to, (*it));
        it.skip_children();
        ++it;
      }
      to = dst.begin();
      it = src.begin();
      while(it != src.end())
      {
        to = dst.replace(to, it);
        to.skip_children();
        it.skip_children();
        ++to;
        ++it;
      }
    }
};

#endif  // _POSEHELPER_HPP_

