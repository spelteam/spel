#ifndef _POSEHELPER_HPP_
#define _POSEHELPER_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif  // WINDOWS

// tree.hh
#include <tree.hh>

#include "limbLabel.hpp"
#include "POSERECT.hpp"
#include "PHPoint.hpp"

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

    static cv::Mat rotateImageToDefault(const cv::Mat &imgSource,
      const POSERECT <cv::Point2f> &initialRect, const float angle,
      const cv::Size &size);
  };

}

#endif  // _POSEHELPER_HPP_
