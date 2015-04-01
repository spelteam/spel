#include "limbLabel.hpp"
#include "poseHelper.hpp"

LimbLabel::LimbLabel()
{
}

LimbLabel::LimbLabel(const LimbLabel& ll)
{
  limbID = ll.getLimbID();
  center = ll.getCenter();
  angle = ll.getAngle();
  scores = ll.getScores();
  polygon = ll.getPolygon();
  isOccluded = ll.getIsOccluded();
  isWeak = ll.getIsWeak();
}

LimbLabel::LimbLabel(int _id, Point2f _centre, float _angle, vector<Point2f> _polygon, vector<Score> _scores, bool _isOccluded)
{
  limbID = _id;
  center = _centre;
  angle = _angle;
  polygon = _polygon;
  scores = _scores;
  isOccluded = _isOccluded;
}
LimbLabel &LimbLabel::operator=(const LimbLabel &ll)
{
  if (this == &ll)
  {
    return *this;
  }
  this->limbID = ll.getLimbID();
  this->center = ll.getCenter();
  this->angle = ll.getAngle();
  this->scores = ll.getScores();
  this->polygon = ll.getPolygon();
  this->isOccluded = ll.getIsOccluded();
  this->isWeak = ll.getIsWeak();
  return *this;
}

Point2f LimbLabel::getCenter(void) const
{
  return center;
}

vector<Score> LimbLabel::getScores(void) const
{
  return scores;
}

int LimbLabel::getLimbID(void) const
{
  return limbID;
}

float LimbLabel::getAngle(void) const
{
  return angle;
}

vector <Point2f> LimbLabel::getPolygon(void) const
{
  return polygon;
}

bool LimbLabel::getIsOccluded(void) const
{
  return isOccluded;
}

bool LimbLabel::getIsWeak(void) const
{
  return isWeak;
}

bool LimbLabel::operator < (const LimbLabel &ll) const
{
  return getAvgScore() < ll.getAvgScore();
}

bool LimbLabel::operator > (const LimbLabel &ll) const
{
  return !(*this < ll);
}

float LimbLabel::getAvgScore(void) const
{
  float sum = 0;
  uint32_t count = 0;
  for_each(scores.begin(), scores.end(), [&](Score s)
  {
    count++;
    sum += s.getScore() * s.getCoeff();
  });
  return ((count > 0) ? (sum / count) : 0.0f);
}

void LimbLabel::getEndpoints(Point2f &p0, Point2f &p1) const
{
    p0 = 0.5*polygon[0]+0.5*polygon[3];
    p1 = 0.5*polygon[1]+0.5*polygon[2];
    return; // this function needs to update the two endpoints passed into it
}

//Point2f LimbLabel::rotatePoint2D(const Point2f point, const Point2f pivot, const float degrees)
//{
//    double radians = degrees*M_PI/180.0;
//    //first center "point"

//    Point2f pt, cnt;
//    pt = point;
//    cnt = pivot;

//    pt = pt-cnt;

//    //now rotate
//    Point2f result;
//    result.x = pt.x*cosf(radians) - pt.y*sinf(radians);
//    result.y = pt.x*sinf(radians) + pt.y*cosf(radians);

//    //now translate back
//    result = result+cnt;

//    Point2i res = result;
//    return res;
//}

bool LimbLabel::containsPoint(Point2f pt)
{
    //decide whether this point belongs to this polygon or not

//    float *vertx = new float[4];
//    vertx[0] = polygon[0].x;
//    vertx[1] = polygon[1].x;
//    vertx[2] = polygon[2].x;
//    vertx[3] = polygon[3].x;

//    float *verty = new float[4];
//    verty[0] = polygon[0].y;
//    verty[1] = polygon[1].y;
//    verty[2] = polygon[2].y;
//    verty[3] = polygon[3].y;

//    int result = LimbLabel::pnpoly(4, vertx, verty, pt.x, pt.y);

    POSERECT <Point2f> poserect(polygon[0], polygon[1], polygon[2], polygon[3]);

    return poserect.containsPoint(pt);
}

////decide whether this point belongs to this polygon or not
////http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html#Originality
//int LimbLabel::pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
//{
//  int i, j, c = 0;
//  for (i = 0, j = nvert-1; i < nvert; j = i++) {
//    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
//     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
//       c = !c;
//  }
//  return c;
//}

void LimbLabel::addScore(Score detectionScore)
{
  scores.push_back(detectionScore);
}
