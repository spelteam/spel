#include "limbLabel.hpp"

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

LimbLabel::LimbLabel(int _id, Point2f _centre, float _angle, vector<Point2f> _polygon, vector<Score> _scores)
{
  limbID = _id;
  center = _centre;
  angle = _angle;
  polygon = _polygon;
  scores = _scores;

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
  return getSumScore() < ll.getSumScore();
}

bool LimbLabel::operator > (const LimbLabel &ll) const
{
  return !(*this < ll);
}

float LimbLabel::getSumScore(void) const
{
  float sum = 0;
  for_each(scores.begin(), scores.end(), [&](Score s)
  {
    sum += s.getScore();
  });
  return sum;
}

void LimbLabel::getEndpoints(Point2f &p0, Point2f &p1) const
{
  
}
