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

namespace SPEL
{
  // Return ROI (rect which include white object with max area on this mask image)
  cv::Rect SearchROI_(cv::Mat mask);

  // ROI transformation: to narrow or extend
  cv::Rect resizeROI_(cv::Rect ROI, cv::Size NewROISize, cv::Size ImageSize = cv::Size(0,0) );

}

#endif;