#ifndef _POSERECT_HPP_
#define _POSERECT_HPP_

#include <vector>

#include "spelHelper.hpp"

namespace SPEL
{
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
}

#endif // _POSERECT_HPP_
