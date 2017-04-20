#ifndef _POSEHELPER_HPP_
#define _POSEHELPER_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif  // WINDOWS
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>

// tree.hh
#include <tree.hh>

#include "limbLabel.hpp"
#include "bodyJoint.hpp"

namespace SPEL
{
  template <class T> class spelRECT;

  /// <summary>
  /// Class with common functions.
  /// </summary>  
  class spelHelper
  {
  public:
    /// <summary>Find squared distance between two points.</summary>
    /// <param name="one">The first point.</param>
    /// <param name="two">The second point.</param>
    /// <returns>Squared distance between one and two.</returns>
    template <typename T>
    static float distSquared(const T &one, const T &two) 
    {
      return pow(one.x - two.x, 2.0f) + pow(one.y - two.y, 2.0f);
    }
    /// <summary>Find squared distance between two points.</summary>
    /// <param name="one">The first point.</param>
    /// <param name="two">The second point.</param>
    /// <returns>Squared distance between one and two.</returns>
    template <typename T>
    static float distSquared3d(const T &one, const T &two) 
    {
      return pow(one.x - two.x, 2.0f) + pow(one.y - two.y, 2.0f) + 
        pow(one.z - two.z, 2.0f);
    }
    /// <summary>Find angle between two vectors on a plane.</summary>
    /// <param name="x1">The x coordinate of the first vector.</param>
    /// <param name="y1">The y coordinate of the first vector.</param>
    /// <param name="x2">The x coordinate of the second vector.</param>
    /// <param name="y2">The y coordinate of the second vector.</param>
    /// <returns>
    /// Angle between first and second vector positive counter clockwise
    /// angle is in the range [-PI; PI]
    /// </returns>   
    template <typename T>
    static T angle2D(const T x1, const T y1, const T x2, const T y2) 
    {
      const auto zero = static_cast<T> (0.0f);
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
      while (dtheta > static_cast<T> (M_PI))
        dtheta -= (static_cast<T> (M_PI) * static_cast<T> (2.0f));
      while (dtheta < static_cast<T> (-M_PI))
        dtheta += (static_cast<T> (M_PI) * static_cast<T> (2.0f));

      return dtheta;
    }
    /// <summary>Interpolates the value.</summary>
    /// <param name="prevAngle">The previous angle.</param>
    /// <param name="nextAngle">The next angle.</param>
    /// <param name="step">The step.</param>
    /// <param name="numSteps">The number of steps.</param>
    /// <returns>The interpolated point.</returns>
    template <typename T, typename D, typename E>
    static T interpolateValue(const T prevAngle, const T nextAngle, 
      const D step, const E numSteps) 
      
    {
      auto t = static_cast<T> (0.0f);
      if (numSteps != 0)
        t = step / static_cast<T>(numSteps);
      return prevAngle * (static_cast<T> (1.0f) - t) + nextAngle * t;
    }    
    /// <summary>Rotates the point.</summary>
    /// <param name="point">The point.</param>
    /// <param name="pivot">The pivot.</param>
    /// <param name="degrees">The angle.</param>
    /// <returns>The rotated point.</returns>
    template <typename T, typename D>
    static T rotatePoint2D(const T &point, const T &pivot, const D degrees) 
      
    {
      auto radians = static_cast<D>(degrees * M_PI / static_cast<D> (180.0f));
      auto cnt = pivot;
      auto pt = point - cnt;
      T result;
      result.x = pt.x * std::cos(radians) - pt.y * std::sin(radians);
      result.y = pt.x * std::sin(radians) + pt.y * std::cos(radians);
      result = result + cnt;
      return result;
    }
    /// <summary>Copies the tree.</summary>
    /// <param name="dst">The destination.</param>
    /// <param name="src">The source.</param>
    template <class T, class tree_node_allocator>
    static void copyTree(tree <T, tree_node_allocator> &dst, 
      const tree <T, tree_node_allocator> &src) 
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
    /// <summary>Recalculates the weak scores.</summary>
    /// <param name="labels">The labels.</param>
    /// <param name="detectorName">Name of the detector.</param>
    /// <param name="standardDiviationTreshold">
    /// The standard diviation treshold.
    /// </param>
    static void RecalculateScoreIsWeak(std::vector <LimbLabel> &labels, 
      const std::string &detectorName, 
      const float standardDiviationTreshold);
    /// <summary>Rotates the image to default.</summary>
    /// <param name="imgSource">The image source.</param>
    /// <param name="initialRect">The initial rect.</param>
    /// <param name="angle">The angle.</param>
    /// <param name="size">The size.</param>
    /// <returns>The rotated image.</returns>
    static cv::Mat rotateImageToDefault(const cv::Mat &imgSource,
      const spelRECT <cv::Point2f> &initialRect, const float angle,
      const cv::Size &size);
    /// <summary>Rounds the specified point.</summary>
    /// <param name="pt">The point.</param>
    /// <returns>The rounded point.</returns>
    static cv::Point2f round(const cv::Point2f& pt);
    /// <summary>Rounds the specified rect.</summary>
    /// <param name="rect">The rect.</param>
    /// <returns>The rounded rect.</returns>
    static spelRECT<cv::Point2f> round(const spelRECT <cv::Point2f> &rect);
    /// <summary>Gets the unique identifier.</summary>
    /// <returns>The unique identifier.</returns>
    static std::string getGUID(void);
    /// <summary>Gets the random string.</summary>
    /// <returns>The random string.</returns>
    static std::string getRandomStr(void);
    /// <summary>Gets the name of the temporary file.</summary>
    /// <param name="extension">The file extension.</param>
    /// <returns>The name of the temporary file.</returns>
    static std::string getTempFileName(const std::string &extension = "");
    /// <summary>Checks if the file exists.</summary>
    /// <param name="file">The file.</param>
    /// <returns>Existance of the file.</returns>
    static bool checkFileExists(const std::string &file);
    /// <summary>Copies the file.</summary>
    /// <param name="dst">The destination.</param>
    /// <param name="src">The source.</param>
    static void copyFile(const std::string &dst, const std::string &src);
    /// <summary>Compares two float numbers.</summary>
    /// <param name="one">The first number.</param>
    /// <param name="two">The second number.</param>
    /// <returns>
    /// -1 if one < two
    /// 0 if one == two
    /// 1 if one > two
    /// </returns>
    template <typename T>
    static int compareFloat(const T one, const T two) 
    {
      if (std::fabs(one - two) <= std::numeric_limits<T>::epsilon())
        return 0;
      return (one < two ? -1 : 1);
    }
    /// <summary>Gets the angle between two points.</summary>
    /// <param name="j0">The first point.</param>
    /// <param name="j1">The second point.</param>
    /// <returns>The angle.</returns>
    static float getAngle(const cv::Point2f &j0, const cv::Point2f &j1)
      ;    
    /// <summary>Gets the angle.</summary>
    /// <param name="point">The point.</param>
    /// <returns>The angle.</returns>
    static float getAngle(const cv::Point2f &point);
    /// <summary>Gets the angle between two <see cref="BodyJoint" />.</summary>
    /// <param name="j0">The first joint.</param>
    /// <param name="j1">The second joint.</param>
    /// <returns>The angle.</returns>
    static float getAngle(const BodyJoint &j0, const BodyJoint &j1);    
    /// <summary>Merges the parameters.</summary>
    /// <param name="dst">The destination.</param>
    /// <param name="src">The source.</param>
    static void mergeParameters(std::map <std::string, float> &dst, const std::map <std::string, float> &src);

    static long clock_to_ms(long t);

    template <class T, class F>
    static F for_each(T t, F f)
    {
      for (auto &i : t)
        f(i);
      return std::move(f);
    }

    template <class T, class D, class F>
    static F for_each(T t, D d, F f)
    {
      for (auto &i : t)
        for (auto &j : d)
          f(i, j);
      return std::move(f);
    }
  };

  /// <summary>
  /// Represents rectangle.
  /// </summary>  
  template <class T>
  class spelRECT
  {
  public:    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelRECT"/> class.
    /// </summary>
    spelRECT(void) 
      : point1(),
      point2(),
      point3(),
      point4() {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelRECT"/> class.
    /// COpy constructor.
    /// </summary>
    /// <param name="poserect">The poserect.</param>
    spelRECT(const spelRECT<T>& poserect) 
      : point1(poserect.point1),
      point2(poserect.point2),
      point3(poserect.point3),
      point4(poserect.point4) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelRECT"/> class.
    /// Move constructor.
    /// </summary>
    /// <param name="poserect">The poserect.</param>
    spelRECT(spelRECT<T>&& poserect) 
      : point1(std::move(poserect.point1)),
      point2(std::move(poserect.point2)),
      point3(std::move(poserect.point3)),
      point4(std::move(poserect.point4)) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelRECT"/> class.
    /// </summary>
    /// <param name="p1">The first point.</param>
    /// <param name="p2">The second point.</param>
    /// <param name="p3">The third point.</param>
    /// <param name="p4">The fourth point.</param>
    spelRECT(const T &p1, const T &p2, const T &p3, const T &p4) 
      : point1(p1),
      point2(p2),
      point3(p3),
      point4(p4) {}
    T point1;
    T point2;
    T point3;
    T point4;
    /// <summary>Determines whether the specified rectangle contains point.</summary>
    /// <param name="point">The point.</param>
    /// <returns>
    /// 1 - point lies in rectangle
    /// -1 - point doesn't lies in rectangle
    /// 0 - point lies on the edge(vertex)
    /// </returns>   
    int8_t containsPoint(const T &point) const 
    {
      std::vector <T> contour;
      contour.push_back(point1);
      contour.push_back(point2);
      contour.push_back(point3);
      contour.push_back(point4);

      return static_cast<int8_t>(pointPolygonTest(contour, point, false));
    }
    /// <summary>Convert rectangle to vector of points.</summary>
    /// <returns>The vector of points.</returns>
    std::vector <T> asVector(void) const 
    {
      std::vector <T> contour;
      contour.push_back(point1);
      contour.push_back(point2);
      contour.push_back(point3);
      contour.push_back(point4);
      return contour;
    }
    /// <summary>Gets the minimum & maximum x & y.</summary>
    /// <param name="minx">The minimum x.</param>
    /// <param name="miny">The minimum y.</param>
    /// <param name="maxx">The maximum x.</param>
    /// <param name="maxy">The maximum y.</param>
    template <typename D>
    void GetMinMaxXY(D &minx, D &miny, D &maxx, D &maxy) const 
    {
      minx = std::min(std::min(point1.x, point2.x), 
        std::min(point3.x, point4.x));
      maxx = std::max(std::max(point1.x, point2.x), 
        std::max(point3.x, point4.x));
      miny = std::min(std::min(point1.y, point2.y), 
        std::min(point3.y, point4.y));
      maxy = std::max(std::max(point1.y, point2.y), 
        std::max(point3.y, point4.y));
    }
    /// <summary>Gets the center point of the rectangle.</summary>
    /// <returns>The center point of the rectangle.</returns>
    template <typename D>
    D GetCenter(void) const
    {
      const auto &center1 = 0.5f * point1 + 0.5f * point3;
      const auto &center2 = 0.5f * point2 + 0.5f * point4;
      if (sqrt(spelHelper::distSquared(center1, center2)) > 0.001f)
        throw std::logic_error("Rect center couldn't be found");
      else
        return center1;
    }    
    /// <summary>Assignment operator.</summary>
    /// <param name="rect">The rect.</param>
    /// <returns>The rect.</returns>
    spelRECT<T>& operator=(const spelRECT<T>& rect) 
    {
      if (&rect == this) return *this;

      point1 = rect.point1;
      point2 = rect.point2;
      point3 = rect.point3;
      point4 = rect.point4;

      return *this;
    }    
    /// <summary>Move operator.</summary>
    /// <param name="rect">The rect.</param>
    /// <returns>The rect</returns>
    spelRECT<T>& operator=(spelRECT<T>&& rect) 
    {
      std::swap(point1, rect.point1);
      std::swap(point2, rect.point2);
      std::swap(point3, rect.point3);
      std::swap(point4, rect.point4);

      return *this;
    }    
    /// <summary>Comparison operator.</summary>
    /// <param name="rect">The rect.</param>
    /// <returns>Result of the comparison.</returns>
    bool operator==(const spelRECT <T> &rect) const 
    {
      return (this->point1 == rect.point1 && this->point2 == rect.point2 && 
        this->point3 == rect.point3 && this->point4 == rect.point4);
    }
    /// <summary>Comparison operator.</summary>
    /// <param name="rect">The rect.</param>
    /// <returns>Result of the comparison.</returns>
    bool operator!=(const spelRECT <T> &rect) const 
    {
      return !(*this == rect);
    }
    /// <summary>Gets the size of the rect.</summary>
    /// <returns>The size of the rect</returns>
    template <typename D>
    D RectSize(void) const 
    {
      return (D(sqrt(spelHelper::distSquared(point2, point3)), 
        sqrt(spelHelper::distSquared(point1, point2))));
    }
  };

  /// <summary>
  /// Represents the OpenCV Point class with the comparer
  /// </summary>
  template <typename T>
  class spelPoint : public cv::Point_ < T >
  {
  public:    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint"/> class.
    /// </summary>
    spelPoint()  : cv::Point_<T>() {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint"/> class.
    /// </summary>
    /// <param name="_x">The x.</param>
    /// <param name="_y">The y.</param>
    spelPoint(T _x, T _y)  : cv::Point_<T>(_x, _y) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    spelPoint(const cv::Point_<T>& pt)  : cv::Point_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    spelPoint(const CvPoint& pt)  : cv::Point_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    spelPoint(const CvPoint2D32f& pt)  : cv::Point_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint"/> class.
    /// </summary>
    /// <param name="sz">The size.</param>
    spelPoint(const cv::Size_<T>& sz)  : cv::Point_<T>(sz) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint"/> class.
    /// </summary>
    /// <param name="v">The vector.</param>
    spelPoint(const cv::Vec<T, 2>& v)  : cv::Point_<T>(v) {}
    /// <summary>Comparison operator.</summary>
    /// <param name="pt">The point.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator < (const spelPoint& pt) const 
    {
      return (this->x < pt.x || (this->x == pt.x && this->y < pt.y));
    }
  };

  /// <summary>
  /// Represents the OpenCV Point3 class with the comparer
  /// </summary>
  template <typename T>
  class spelPoint3 : public cv::Point3_ < T >
  {
  public:    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint3"/> class.
    /// </summary>
    spelPoint3()  : cv::Point3_<T>() {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint3"/> class.
    /// </summary>
    /// <param name="_x">The x.</param>
    /// <param name="_y">The y.</param>
    /// <param name="_z">The z.</param>
    spelPoint3(T _x, T _y, T _z)  : cv::Point3_<T>(_x, _y, _z) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint3"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    spelPoint3(const cv::Point3_<T>& pt)  : cv::Point3_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint3"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    explicit spelPoint3(const cv::Point_<T>& pt)  : cv::Point3_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint3"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    spelPoint3(const CvPoint3D32f& pt)  : cv::Point3_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="spelPoint3"/> class.
    /// </summary>
    /// <param name="v">The vector.</param>
    spelPoint3(const cv::Vec<T, 3>& v)  : cv::Point3_<T>(v) {}    
    /// <summary>Comparison operator.</summary>
    /// <param name="pt">The point.</param>
    /// <returns>The comparison result.</returns>
    bool operator < (const spelPoint3& pt) const 
    {
      return (this->x < pt.x || (this->x == pt.x && this->y < pt.y) || 
        (this->x == pt.x && this->y == pt.y && this->z < pt.z));
    }
  };

}

#endif  // _POSEHELPER_HPP_
