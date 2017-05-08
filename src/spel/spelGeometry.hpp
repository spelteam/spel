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

#include "frame.hpp"
#include "solvlet.hpp"
#include "skeleton.hpp"
#include "imagepixelsimilaritymatrix.hpp"

namespace SPEL
{
//ROI
  // Return ROI (endpoints of the rect which include white object with max area on this mask image)
  std::vector<cv::Point2i> SearchROI(cv::Mat mask);
  cv::Rect toROIRect(std::vector<cv::Point2i> endpoints);
  cv::Rect toROIRect(std::vector<cv::Point2f> endpoints);

  // Search center of white figure in the ROI of cv::Mat<UC_8UC1> image
  cv::Point2f MaskCenter(cv::Mat mask, std::vector<cv::Point2i> ROIEndpoints, uchar colorThreshold = 9);
  cv::Point2f MaskCenter(cv::Mat mask, cv::Rect ROIRect, uchar colorThreshold = 9);

  // ROI transformation: to narrow or extend
  cv::Rect resizeROI_(cv::Rect ROI, cv::Size NewROISize, cv::Size ImageSize = cv::Size(0,0) );
  bool correctROI(cv::Rect &ROI, cv::Size imageSize);
  bool correctEndpoints(std::vector<cv::Point2f> &endpoints, cv::Size imageSize);

//Part polygon
  bool isPartPolygon(std::vector<cv::Point2f> partPolygon, float error = 0.01);
  std::vector<cv::Point2f> buildPartPolygon(float LWRatio, cv::Point2f p0, cv::Point2f p1);
  float getPartLenght(std::vector<cv::Point2f> partPolygon);
  float getPartWidth(std::vector<cv::Point2f> partPolygon);
  cv::Point2f getPartCenter(std::vector<cv::Point2f> partPolygon);
  cv::Point2f getCenter(std::vector<cv::Point2f> polygon);
  std::map<int, std::vector<cv::Point2f>> getAllPolygons(Skeleton skeleton);
  std::vector<cv::Point2f> getEndpoints(std::vector<cv::Point2f> polygon);
  std::vector<cv::Point2f> getEndpoints(std::map<int, std::vector<cv::Point2f>> polygons);
  std::vector<cv::Point2f> getEndpoints(Skeleton skeleton);
  std::map<uint32_t,std::vector<LimbLabel>> createLabels(Skeleton & skeleton);

//Skeleton
  Skeleton operator+(Skeleton s1, Skeleton s2);
  Skeleton operator+(Skeleton s, cv::Point2f P);
  Skeleton operator+(cv::Point2f P, Skeleton s);
  Skeleton operator-(Skeleton s1, cv::Point2f P);
  Skeleton operator*(Skeleton s, float k);
  Skeleton operator*(float k, Skeleton s);
  Skeleton operator/(Skeleton s, float k);
  cv::Point2f SkeletonCenter(std::map<int, std::vector<cv::Point2f>> polygons);
  cv::Point2f SkeletonCenter(Skeleton skeleton);
  // Copiyng shifted skeleton from neighbor frame
  void setSkeleton(Frame* frame, Frame* neighborFrame);

// Points
  bool inside(cv::Point2f p, cv::Size imageSize);
  bool inside(cv::Point p, cv::Size imageSize);

//Interpolation
  // Remove lockframe skeleton
  void clearSkeleton(Frame &frame);
  void clearSkeleton(Frame* frame);
  // Remove skeletons from all lockframes
  void clearSkeletons(std::vector<Frame*> frames);
  // Rough interpolation. Only slice which has a keyframes on the his beginning and end positions will interpolated.
  void interpolate(std::vector<Frame*> slice); // bad interpolation!?
  // Create interpolated skeleton, if "useKeyframesOnly == false && replaceExisting == false" - then all existing skeletons will used as keyframes and interpolation will created only for empty skeletons.
  void interpolate2(std::vector<Frame*> slice, bool useKeyframesOnly = true, bool replaceExisting = true); // works on short slices (slices without redirecting motion)
  // Interpolation by ISM, creating skeleton as average from similary keyframes
  std::vector<int> interpolate3(std::vector<Frame*> frames, ImagePixelSimilarityMatrix* MSM = 0, float SimilarityThreshold = 0.55f, bool replaceExisting = true);
  // Interpolation by ISM, creating skeleton as average from similary keyframes
  std::vector<int> interpolate4(std::vector<Frame*> frames, ImagePixelSimilarityMatrix* MSM = 0, float SimilarityThreshold = 0.45f);
  // Copy similary keyframes as interpolation
  std::vector<int> propagateKeyFrames(std::vector<Frame*> frames, ImagePixelSimilarityMatrix* MSM = 0, float SimilarityThreshold = 0.55f, bool replaceExisting = true);
  // Copy similary frames as interpolation
  std::vector<int> propagateFrames(std::vector<Frame*> frames, ImagePixelSimilarityMatrix* MSM = 0, float SimilarityThreshold = 0.55f, bool replaceExisting = false);
  // Compare mask and skeleton mask: score 1.0f = 100%, 0.0f = 0% coincidence
  float skeletonScore(cv::Mat mask, Skeleton skeleton, cv::Point2f skeletonShift = cv::Point2f(0.0f, 0.0f));

//Visualization
  void putPartRect(cv::Mat image, std::vector<cv::Point2f> polygon, cv::Scalar color = cv::Scalar(255, 255, 255));
  void putSkeleton(cv::Mat image, Skeleton skeleton, cv::Scalar color = cv::Scalar(255, 255, 255));
  void putSkeletonMask(cv::Mat &mask, Skeleton skeleton, cv::Size maskSize = cv::Size(0, 0), uchar color = 255);
  void putLabels(cv::Mat image, std::vector<LimbLabel> frameLabels, cv::Scalar color = cv::Scalar(255, 255, 255));
  std::string to_string(int num, uchar length);

}

#endif;