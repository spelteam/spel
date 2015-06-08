#ifndef _LIBPOSE_LIMBLABEL_HPP_
#define _LIBPOSE_LIMBLABEL_HPP_

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif  // WINDOWS

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cstdint>
#include "score.hpp"

using namespace std;
using namespace cv;

class LimbLabel
{
public:
  //TODO (Vitaliy Koshura): Need implementation
  LimbLabel();
  LimbLabel(const LimbLabel& ll);
  LimbLabel(int _id, Point2f _centre, float _angle, vector<Point2f> _polygon, vector<Score> _scores, bool _isOccluded = false);

  /// output labels as printable string
  //TODO (Vitaliy Koshura): Need implementation
  string toString();
  LimbLabel & operator = (const LimbLabel &ll);
  //TODO (Vitaliy Koshura): Need implementation
  bool operator == (const LimbLabel &ll) const;
  //TODO (Vitaliy Koshura): Need implementation
  bool operator != (const LimbLabel &ll) const;
  bool operator < (const LimbLabel &ll) const;
  bool operator > (const LimbLabel &ll) const;
  // compute the endpoints of the limb that this label would produce
  //TODO (Vitaliy Koshura): Need implementation
  void getEndpoints(Point2f &p0, Point2f &p1) const;
  //TODO (Vitaliy Koshura): Need implementation
  void addScore(Score detectionScore);
  Point2f getCenter(void) const;
  ///get the label scores
  vector<Score> getScores(void) const;
  void setScores(vector <Score> _scores);
  int getLimbID(void) const;
  float getAngle(void) const;
  vector <Point2f> getPolygon(void) const;
  bool getIsOccluded(void) const;
  float getAvgScore(bool bNegativeToPositive = false) const;
  void Resize(float factor);
  bool containsPoint(Point2f pt);
private:
#ifdef DEBUG
  FRIEND_TEST(nskpsolverTests, ScoreCostAndJointCost); // Used for set isWeak to "false" in nskpsolver_tests.computeScoreCost
  FRIEND_TEST(nskpsolverTests, evaluateSolution);// Used for set isWeak to "false" in nskpsolver_tests.evaluateSolution
#endif  // DEBUG
  ////rotate a point around a pivot by degrees
  //    Point2f rotatePoint2D(const Point2f point, const Point2f pivot, const float degrees);
  /// Label center 
  Point2f center;
  /// identifier representing what bone this label belongs to
  int limbID;
  /// degrees rotation from y=0 (horizontal line)
  float angle;
  /// detector scores
  vector<Score> scores;
  /// polygon for the label (already rotated, scaled etc. - from which scores are generated)
  vector<Point2f> polygon;
  /// signify label is for an occluded limb
  bool isOccluded;
};

#endif  // _LIBPOSE_LIMBLABEL_HPP_

