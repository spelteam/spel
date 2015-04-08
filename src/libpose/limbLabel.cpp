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

bool LimbLabel::containsPoint(Point2f pt)
{
    //decide whether this point belongs to this polygon or not

    POSERECT <Point2f> poserect(polygon[0], polygon[1], polygon[2], polygon[3]);

    int result = poserect.containsPoint(pt);

    if(result!=-1)
        return true;
    else
        return false;

    return false;
}

void LimbLabel::addScore(Score detectionScore)
{
  scores.push_back(detectionScore);
}
