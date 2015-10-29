#include "limbLabel.hpp"
#include "spelHelper.hpp"

namespace SPEL
{
  LimbLabel::LimbLabel()
  {
    limbID = -1;
    center = cv::Point2f(0, 0);
    angle = 0;
    scores = std::vector<Score>();
    polygon = std::vector<cv::Point2f>();
    for (int i = 0; i < 4; ++i)
      polygon.push_back(cv::Point2f(0, 0));
    isOccluded = true;
  }

  LimbLabel::LimbLabel(const LimbLabel& ll)
  {
    limbID = ll.getLimbID();
    center = ll.getCenter();
    angle = ll.getAngle();
    scores = ll.getScores();
    polygon = ll.getPolygon();
    isOccluded = ll.getIsOccluded();
  }

  LimbLabel::LimbLabel(int _id, cv::Point2f _centre, float _angle, std::vector<cv::Point2f> _polygon, std::vector<Score> _scores, bool _isOccluded)
  {
    limbID = _id;
    center = _centre;
    angle = _angle;
    polygon = _polygon;
    scores = _scores;
    isOccluded = _isOccluded;
  }

  LimbLabel::~LimbLabel(void)
  {
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
    return *this;
  }

  bool LimbLabel::operator==(const LimbLabel &ll) const
  {
    return (this->limbID == ll.getLimbID() && this->center == ll.getCenter() && this->angle == ll.getAngle());
  }

  bool LimbLabel::operator!=(const LimbLabel &ll) const
  {
    return !(*this == ll);
  }

  cv::Point2f LimbLabel::getCenter(void) const
  {
    return center;
  }

  std::vector<Score> LimbLabel::getScores(void) const
  {
    return scores;
  }

  void LimbLabel::setScores(std::vector <Score> _scores)
  {
    scores = _scores;
  }

  int LimbLabel::getLimbID(void) const
  {
    return limbID;
  }

  float LimbLabel::getAngle(void) const
  {
    return angle;
  }

  std::vector <cv::Point2f> LimbLabel::getPolygon(void) const
  {
    return polygon;
  }

  bool LimbLabel::getIsOccluded(void) const
  {
    return isOccluded;
  }

  bool LimbLabel::operator < (const LimbLabel &ll) const
  {
    return getAvgScore(true) < ll.getAvgScore(true); //true?
  }

  bool LimbLabel::operator > (const LimbLabel &ll) const
  {
    return getAvgScore(true) > ll.getAvgScore(true); //true?
  }

  float LimbLabel::getAvgScore(bool bNegativeToPositive) const
  {
    //@TODO: fix if multiple scores are added
    float sum = 0;
    float count = 0;
    for_each(scores.begin(), scores.end(), [&](Score s)
    {
      count++;
      sum += (bNegativeToPositive ? std::abs(s.getScore()) : s.getScore()) * s.getCoeff();
    });

    //float result = ((count > 0) ? (sum / count) : 0.0f);
    return sum;
  }

  void LimbLabel::getEndpoints(cv::Point2f &p0, cv::Point2f &p1) const
  {
    //0-1 R, 1-2 G, 2-3 B, 3-0 P
    p0 = 0.5*polygon[3] + 0.5*polygon[0]; //pink = parent
    p1 = 0.5*polygon[1] + 0.5*polygon[2]; //green = child
    
    return; // this function needs to update the two endpoints passed into it
  }

  bool LimbLabel::containsPoint(cv::Point2f pt)
  {
    //decide whether this point belongs to this polygon or not

    POSERECT <cv::Point2f> poserect(polygon[0], polygon[1], polygon[2], polygon[3]);

    return (poserect.containsPoint(pt) != -1);
  }

  void LimbLabel::addScore(Score detectionScore)
  {
    scores.push_back(detectionScore);
  }

  std::string LimbLabel::toString()
  {
    std::string retString = "";
    retString += std::to_string(limbID) + " ";
    retString += std::to_string(center.x) + " " + std::to_string(center.y) + " ";
    retString += std::to_string(angle) + " ";
    cv::Point2f c0, c1;
    this->getEndpoints(c0, c1);
    retString += std::to_string(c0.x) + " " + std::to_string(c0.y) + " ";
    retString += std::to_string(c1.x) + " " + std::to_string(c1.y) + " ";
    retString += std::to_string(isOccluded) + " ";
    for (int i = 0; i < scores.size(); ++i)
    {
      retString += scores[i].getDetName() + " ";
      retString += std::to_string(scores[i].getScore()) + " ";
    }

    return retString;
  }

  void LimbLabel::Resize(float factor)
  {
    center.x *= factor;
    center.y *= factor;
    for (std::vector <cv::Point2f>::iterator p = polygon.begin(); p != polygon.end(); ++p)
    {
      p->x *= factor;
      p->y *= factor;
    }
  }

}
