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

namespace SPEL
{
  template <class T> class POSERECT;

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
    static float distSquared(const T &one, const T &two) noexcept
    {
      return pow(one.x - two.x, 2.0f) + pow(one.y - two.y, 2.0f);
    }
    /// <summary>Find squared distance between two points.</summary>
    /// <param name="one">The first point.</param>
    /// <param name="two">The second point.</param>
    /// <returns>Squared distance between one and two.</returns>
    template <typename T>
    static float distSquared3d(const T &one, const T &two) noexcept
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
    static T angle2D(const T x1, const T y1, const T x2, const T y2) noexcept
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
      noexcept
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
      noexcept
    {
      auto radians = static_cast<D>(degrees * M_PI / static_cast<D> (180.0f));
      auto cnt = pivot;
      auto pt = point - cnt;
      T result;
      result.x = pt.x * cosf(radians) - pt.y * sinf(radians);
      result.y = pt.x * sinf(radians) + pt.y * cosf(radians);
      result = result + cnt;
      return result;
    }
    /// <summary>Copies the tree.</summary>
    /// <param name="dst">The destination.</param>
    /// <param name="src">The source.</param>
    template <class T, class tree_node_allocator>
    static void copyTree(tree <T, tree_node_allocator> &dst, 
      const tree <T, tree_node_allocator> &src) noexcept
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
      const float standardDiviationTreshold) noexcept;
    /// <summary>Rotates the image to default.</summary>
    /// <param name="imgSource">The image source.</param>
    /// <param name="initialRect">The initial rect.</param>
    /// <param name="angle">The angle.</param>
    /// <param name="size">The size.</param>
    /// <returns>The rotated image.</returns>
    static cv::Mat rotateImageToDefault(const cv::Mat &imgSource,
      const POSERECT <cv::Point2f> &initialRect, const float angle,
      const cv::Size &size);
    /// <summary>Rounds the specified point.</summary>
    /// <param name="pt">The point.</param>
    /// <returns>The rounded point.</returns>
    static cv::Point2f round(const cv::Point2f& pt) noexcept;
    /// <summary>Rounds the specified rect.</summary>
    /// <param name="rect">The rect.</param>
    /// <returns>The rounded rect.</returns>
    static POSERECT<cv::Point2f> round(const POSERECT <cv::Point2f> &rect) 
      noexcept;
    /// <summary>Gets the unique identifier.</summary>
    /// <returns>The unique identifier.</returns>
    static std::string getGUID(void) noexcept;
    /// <summary>Gets the random string.</summary>
    /// <returns>The random string.</returns>
    static std::string getRandomStr(void) noexcept;
    /// <summary>Gets the name of the temporary file.</summary>
    /// <param name="extension">The file extension.</param>
    /// <returns>The name of the temporary file.</returns>
    static std::string getTempFileName(const std::string &extension = "");
    /// <summary>Checks if the file exists.</summary>
    /// <param name="file">The file.</param>
    /// <returns>Existance of the file.</returns>
    static bool checkFileExists(const std::string &file) noexcept;
    /// <summary>Copies the file.</summary>
    /// <param name="dst">The destination.</param>
    /// <param name="src">The source.</param>
    static void copyFile(const std::string &dst, const std::string &src);
  };

  /// <summary>
  /// Represents rectangle.
  /// </summary>  
  template <class T>
  class POSERECT
  {
  public:    
    /// <summary>
    /// Initializes a new instance of the <see cref="POSERECT"/> class.
    /// </summary>
    POSERECT(void) noexcept
      : point1(),
      point2(),
      point3(),
      point4() {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="POSERECT"/> class.
    /// COpy constructor.
    /// </summary>
    /// <param name="poserect">The poserect.</param>
    POSERECT(const POSERECT<T>& poserect) noexcept
      : point1(poserect.point1),
      point2(poserect.point2),
      point3(poserect.point3),
      point4(poserect.point4) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="POSERECT"/> class.
    /// Move constructor.
    /// </summary>
    /// <param name="poserect">The poserect.</param>
    POSERECT(POSERECT<T>&& poserect) noexcept
      : point1(std::move(poserect.point1)),
      point2(std::move(poserect.point2)),
      point3(std::move(poserect.point3)),
      point4(std::move(poserect.point4)) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="POSERECT"/> class.
    /// </summary>
    /// <param name="p1">The first point.</param>
    /// <param name="p2">The second point.</param>
    /// <param name="p3">The third point.</param>
    /// <param name="p4">The fourth point.</param>
    POSERECT(const T &p1, const T &p2, const T &p3, const T &p4) noexcept
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
    int8_t containsPoint(const T &point) const noexcept
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
    std::vector <T> asVector(void) const noexcept
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
    void GetMinMaxXY(D &minx, D &miny, D &maxx, D &maxy) const noexcept
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
    POSERECT<T>& operator=(const POSERECT<T>& rect) noexcept
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
    POSERECT<T>& operator=(POSERECT<T>&& rect) noexcept
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
    bool operator==(const POSERECT <T> &rect) const noexcept
    {
      return (this->point1 == rect.point1 && this->point2 == rect.point2 && 
        this->point3 == rect.point3 && this->point4 == rect.point4);
    }
    /// <summary>Comparison operator.</summary>
    /// <param name="rect">The rect.</param>
    /// <returns>Result of the comparison.</returns>
    bool operator!=(const POSERECT <T> &rect) const noexcept
    {
      return !(*this == rect);
    }
    /// <summary>Gets the size of the rect.</summary>
    /// <returns>The size of the rect</returns>
    template <typename D>
    D RectSize(void) const noexcept
    {
      return (D(sqrt(spelHelper::distSquared(point2, point3)), 
        sqrt(spelHelper::distSquared(point1, point2))));
    }
  };

  /// <summary>
  /// Represents the OpenCV Point class with the comparer
  /// </summary>
  template <typename T>
  class PHPoint : public cv::Point_ < T >
  {
  public:    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint"/> class.
    /// </summary>
    PHPoint() noexcept : cv::Point_<T>() {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint"/> class.
    /// </summary>
    /// <param name="_x">The x.</param>
    /// <param name="_y">The y.</param>
    PHPoint(T _x, T _y) noexcept : cv::Point_<T>(_x, _y) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    PHPoint(const cv::Point_<T>& pt) noexcept : cv::Point_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    PHPoint(const CvPoint& pt) noexcept : cv::Point_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    PHPoint(const CvPoint2D32f& pt) noexcept : cv::Point_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint"/> class.
    /// </summary>
    /// <param name="sz">The size.</param>
    PHPoint(const cv::Size_<T>& sz) noexcept : cv::Point_<T>(sz) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint"/> class.
    /// </summary>
    /// <param name="v">The vector.</param>
    PHPoint(const cv::Vec<T, 2>& v) noexcept : cv::Point_<T>(v) {}
    /// <summary>Comparison operator.</summary>
    /// <param name="pt">The point.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator < (const PHPoint& pt) const noexcept
    {
      return (this->x < pt.x || (this->x == pt.x && this->y < pt.y));
    }
  };

  /// <summary>
  /// Represents the OpenCV Point3 class with the comparer
  /// </summary>
  template <typename T>
  class PHPoint3 : public cv::Point3_ < T >
  {
  public:    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint3"/> class.
    /// </summary>
    PHPoint3() noexcept : cv::Point3_<T>() {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint3"/> class.
    /// </summary>
    /// <param name="_x">The x.</param>
    /// <param name="_y">The y.</param>
    /// <param name="_z">The z.</param>
    PHPoint3(T _x, T _y, T _z) noexcept : cv::Point3_<T>(_x, _y, _z) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint3"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    PHPoint3(const cv::Point3_<T>& pt) noexcept : cv::Point3_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint3"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    explicit PHPoint3(const cv::Point_<T>& pt) noexcept : cv::Point3_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint3"/> class.
    /// </summary>
    /// <param name="pt">The point.</param>
    PHPoint3(const CvPoint3D32f& pt) noexcept : cv::Point3_<T>(pt) {}    
    /// <summary>
    /// Initializes a new instance of the <see cref="PHPoint3"/> class.
    /// </summary>
    /// <param name="v">The vector.</param>
    PHPoint3(const cv::Vec<T, 3>& v) noexcept : cv::Point3_<T>(v) {}    
    /// <summary>Comparison operator.</summary>
    /// <param name="pt">The point.</param>
    /// <returns>The comparison result.</returns>
    bool operator < (const PHPoint3& pt) const noexcept
    {
      return (this->x < pt.x || (this->x == pt.x && this->y < pt.y) || 
        (this->x == pt.x && this->y == pt.y && this->z < pt.z));
    }
  };

}

#endif  // _POSEHELPER_HPP_
