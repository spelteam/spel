#include "limbLabel.hpp"
#include "spelHelper.hpp"

namespace SPEL
{
  LimbLabel::LimbLabel() noexcept
  {
    limbID = -1;
    center = cv::Point2f(0, 0);
    angle = 0;
    scores = std::vector<Score>();
    polygon = std::vector<cv::Point2f>();
    polygon.reserve(4);
    for (auto i = 0; i < 4; ++i)
      polygon.push_back(cv::Point2f(0, 0));
    isOccluded = true;
  }

  LimbLabel::LimbLabel(const LimbLabel& ll) noexcept
    : limbID(ll.getLimbID()),
    center(ll.getCenter()),
    angle(ll.getAngle()),
    scores(ll.getScores()),
    polygon(ll.getPolygon()),
    isOccluded(ll.getIsOccluded())
  {
  }

  LimbLabel::LimbLabel(LimbLabel && ll) noexcept
    : limbID(std::move(ll.getLimbID())),
    center(std::move(ll.getCenter())),
    angle(std::move(ll.getAngle())),
    scores(std::move(ll.getScores())),
    polygon(std::move(ll.getPolygon())),
    isOccluded(std::move(ll.getIsOccluded()))
  {
  }

  LimbLabel::LimbLabel(int _id, cv::Point2f _centre, float _angle, std::vector<cv::Point2f> _polygon, std::vector<Score> _scores, bool _isOccluded) noexcept
  {
    limbID = _id;
    center = _centre;
    angle = _angle;
    polygon = _polygon;
    scores = _scores;
    isOccluded = _isOccluded;
  }

  LimbLabel::~LimbLabel(void) noexcept
  {
  }

  LimbLabel &LimbLabel::operator=(const LimbLabel &ll) noexcept
  {
    if (this == &ll)
      return *this;
    limbID = ll.getLimbID();
    center = ll.getCenter();
    angle = ll.getAngle();
    scores = ll.getScores();
    polygon = ll.getPolygon();
    isOccluded = ll.getIsOccluded();
    return *this;
  }

  LimbLabel & LimbLabel::operator=(LimbLabel && ll) noexcept
  {
    limbID = std::move(ll.getLimbID());
    center = std::move(ll.getCenter());
    angle = std::move(ll.getAngle());
    scores = std::move(ll.getScores());
    polygon = std::move(ll.getPolygon());
    isOccluded = std::move(ll.getIsOccluded());

    return *this;
  }

  bool LimbLabel::operator==(const LimbLabel &ll) const noexcept
  {
    return (limbID == ll.getLimbID() && center == ll.getCenter() && angle == ll.getAngle());
  }

  bool LimbLabel::operator!=(const LimbLabel &ll) const noexcept
  {
    return !(*this == ll);
  }

  cv::Point2f LimbLabel::getCenter(void) const noexcept
  {
    return center;
  }

  std::vector<Score> LimbLabel::getScores(void) const noexcept
  {
    return scores;
  }

  void LimbLabel::setScores(std::vector <Score> _scores) noexcept
  {
    scores = _scores;
  }

  int LimbLabel::getLimbID(void) const noexcept
  {
    return limbID;
  }

  float LimbLabel::getAngle(void) const noexcept
  {
    return angle;
  }

  std::vector <cv::Point2f> LimbLabel::getPolygon(void) const noexcept
  {
    return polygon;
  }

  bool LimbLabel::getIsOccluded(void) const noexcept
  {
    return isOccluded;
  }

  bool LimbLabel::operator < (const LimbLabel &ll) const noexcept
  {
    return getAvgScore(true) < ll.getAvgScore(true); //true?
  }

  bool LimbLabel::operator > (const LimbLabel &ll) const noexcept
  {
    return getAvgScore(true) > ll.getAvgScore(true); //true?
  }

  float LimbLabel::getAvgScore(bool bNegativeToPositive) const noexcept
  {
    //@TODO: fix if multiple scores are added
    auto sum = 0.0f;
    //auto count = 0.0f;
    for_each(scores.begin(), scores.end(), [&](const auto &s)
    {
      //count++;
      sum += (bNegativeToPositive ? std::abs(s.getScore()) : s.getScore()) * s.getCoeff();
    });

    //float result = ((count > 0) ? (sum / count) : 0.0f);
    return sum;
  }

  void LimbLabel::getEndpoints(cv::Point2f &p0, cv::Point2f &p1) const
  {
    const auto err = "Polygon should contain only 4 elements";
    if (polygon.size() != 4)
    {
      DebugMessage(err, 1);
      if (polygon.size() < 4)
        throw std::out_of_range(err);
      else
        throw std::logic_error(err);
    }
    //0-1 R, 1-2 G, 2-3 B, 3-0 P
    p0 = 0.5*polygon.at(3) + 0.5*polygon.at(0); //pink = parent
    p1 = 0.5*polygon.at(1) + 0.5*polygon.at(2); //green = child

    return; // this function needs to update the two endpoints passed into it
  }

  bool LimbLabel::containsPoint(cv::Point2f pt) const
  {
    //decide whether this point belongs to this polygon or not

    const auto err = "Polygon should contain only 4 elements";
    if (polygon.size() != 4)
    {
      DebugMessage(err, 1);
      if (polygon.size() < 4)
        throw std::out_of_range(err);
      else
        throw std::logic_error(err);
    }
    POSERECT <cv::Point2f> poserect(polygon.at(0), polygon.at(1), polygon.at(2), polygon.at(3));

    return (poserect.containsPoint(pt) != -1);
  }

  void LimbLabel::addScore(Score detectionScore) noexcept
  {
    scores.push_back(detectionScore);
  }

  std::string LimbLabel::toString() const noexcept
  {
    auto retString = std::string("");
    retString += std::to_string(limbID) + " ";
    retString += std::to_string(center.x) + " " + std::to_string(center.y) + " ";
    retString += std::to_string(angle) + " ";
    cv::Point2f c0, c1;
    getEndpoints(c0, c1);
    retString += std::to_string(c0.x) + " " + std::to_string(c0.y) + " ";
    retString += std::to_string(c1.x) + " " + std::to_string(c1.y) + " ";
    retString += std::to_string(isOccluded) + " ";
    for (const auto &i : scores)
    {
      retString += i.getDetName() + " ";
      retString += std::to_string(i.getScore()) + " ";
    }
    return retString;
  }

  void LimbLabel::Resize(float factor) noexcept
  {
    center.x *= factor;
    center.y *= factor;
    for (auto &p : polygon)
    {
      p.x *= factor;
      p.y *= factor;
    }
  }

}
