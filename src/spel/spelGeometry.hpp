#ifndef __SPELGEOMETRY_HPP_
#define __SPELGEOMETRY_HPP_

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

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

#include "skeleton.hpp"

namespace SPEL
{
//ROI
  // Return ROI (endpoints of the rect which include white object with max area on this mask image)
  std::vector<cv::Point2i> SearchROI(cv::Mat mask);
  cv::Rect toROIRect(std::vector<cv::Point2i> endpoints);

  // Search center of white figure in the ROI of cv::Mat<UC_8UC1> image
  cv::Point2i MaskCenter(cv::Mat mask, std::vector<cv::Point2i> ROIEndpoints, uchar colorThreshold = 9);
  cv::Point2i MaskCenter(cv::Mat mask, cv::Rect ROIRect, uchar colorThreshold = 9);

  // ROI transformation: to narrow or extend
  cv::Rect resizeROI_(cv::Rect ROI, cv::Size NewROISize, cv::Size ImageSize = cv::Size(0,0) );

//Part polygon
  std::vector<cv::Point2f> buildPartPolygon(float LWRatio, cv::Point2f p0, cv::Point2f p1);
  float getLenght(std::vector<cv::Point2f> partPolygon);
  float getWidth(std::vector<cv::Point2f> partPolygon);
  std::map<int, std::vector<cv::Point2f>> getAllPolygons(Skeleton &skeleton);

//Skeleton
  Skeleton operator+(Skeleton s1, Skeleton s2);
  Skeleton operator+(Skeleton s, cv::Point2f P);
  Skeleton operator+(cv::Point2f P, Skeleton s);
  Skeleton operator-(Skeleton s1, cv::Point2f P);
  Skeleton operator*(Skeleton s, float k);
  Skeleton operator/(Skeleton s, float k);

//Visualization
  void putPartRect(cv::Mat &Image, std::vector<cv::Point2f> polygon, cv::Scalar color = cv::Scalar(255, 255, 255));
  void putSkeleton(cv::Mat image, Skeleton skeleton, cv::Scalar color = cv::Scalar(255, 255, 255));
}

#endif;