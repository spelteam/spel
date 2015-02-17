#ifndef _LIBPOSE_LIMBLABEL_HPP_
#define _LIBPOSE_LIMBLABEL_HPP_

#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif  // WINDOWS

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "score.hpp"

using namespace std;
using namespace cv;

class LimbLabel
{
  public:
//TODO (Vitaliy Koshura): Need implementation
    LimbLabel();
    LimbLabel(const LimbLabel& ll);
    LimbLabel(int _id, Point2f _centre, float _angle, vector<Point2f> _polygon, vector<Score> _scores);

// output labels as printable string
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
    void addScore(Score detectionScore) const; 
    Point2f getCenter(void) const;
    vector<Score> getScores(void) const; //get the label scores
    int getLimbID(void) const;
    float getAngle(void) const;
    vector <Point2f> getPolygon(void) const;
    bool getIsOccluded(void) const;
    bool getIsWeak(void) const;
    float getSumScore(void) const;

    bool containsPoint(Point2f pt);
  private:
////rotate a point around a pivot by degrees
//    Point2f rotatePoint2D(const Point2f point, const Point2f pivot, const float degrees);
// Label center 
    Point2f center;
// identifier representing what bone this label belongs to
    int limbID;
// degrees rotation from y=0 (horizontal line)
    float angle;
// detector scores
    vector<Score> scores;
// polygon for the label (already rotated, scaled etc. - from which scores are generated)
    vector<Point2f> polygon;
// signify label is for an occluded limb
    bool isOccluded;
// signify label is from a badly localised part
    bool isWeak; 
//point in polygon test
    int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy);
};

#endif  // _LIBPOSE_LIMBLABEL_HPP_

