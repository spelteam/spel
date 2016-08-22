#include "limbLabel.hpp"
#include "spelHelper.hpp"

namespace SPEL
{
  LimbLabel::LimbLabel(void) noexcept
  {
    m_limbID = -1;
    m_center = cv::Point2f(0, 0);
    m_angle = 0;
    m_scores = std::vector<Score>();
    m_polygon = std::vector<cv::Point2f>();
    m_polygon.reserve(4);
    for (auto i = 0; i < 4; ++i)
      m_polygon.push_back(cv::Point2f(0, 0));
    m_isOccluded = true;
  }

  LimbLabel::LimbLabel(const LimbLabel& limbLabel) noexcept
    : m_limbID(limbLabel.getLimbID()),
    m_center(limbLabel.getCenter()),
    m_angle(limbLabel.getAngle()),
    m_scores(limbLabel.getScores()),
    m_polygon(limbLabel.getPolygon()),
    m_isOccluded(limbLabel.getIsOccluded())
  {
  }

  LimbLabel::LimbLabel(LimbLabel && limbLabel) noexcept
    : m_limbID(std::move(limbLabel.getLimbID())),
    m_center(std::move(limbLabel.getCenter())),
    m_angle(std::move(limbLabel.getAngle())),
    m_scores(std::move(limbLabel.getScores())),
    m_polygon(std::move(limbLabel.getPolygon())),
    m_isOccluded(std::move(limbLabel.getIsOccluded()))
  {
  }

  LimbLabel::LimbLabel(int id, cv::Point2f center, float angle,
    std::vector<cv::Point2f> polygon, std::vector<Score> scores,
    bool isOccluded) noexcept
  {
    m_limbID = id;
    m_center = center;
    m_angle = angle;
    m_polygon = polygon;
    m_scores = scores;
    m_isOccluded = isOccluded;
  }

  LimbLabel::~LimbLabel(void) noexcept
  {
  }

  LimbLabel &LimbLabel::operator=(const LimbLabel &limbLabel) noexcept
  {
    if (this == &limbLabel)
      return *this;
    m_limbID = limbLabel.getLimbID();
    m_center = limbLabel.getCenter();
    m_angle = limbLabel.getAngle();
    m_scores = limbLabel.getScores();
    m_polygon = limbLabel.getPolygon();
    m_isOccluded = limbLabel.getIsOccluded();
    return *this;
  }

  LimbLabel & LimbLabel::operator=(LimbLabel && limbLabel) noexcept
  {
    m_limbID = std::move(limbLabel.getLimbID());
    m_center = std::move(limbLabel.getCenter());
    m_angle = std::move(limbLabel.getAngle());
    m_scores = std::move(limbLabel.getScores());
    m_polygon = std::move(limbLabel.getPolygon());
    m_isOccluded = std::move(limbLabel.getIsOccluded());

    return *this;
  }

  bool LimbLabel::operator==(const LimbLabel &limbLabel) const noexcept
  {
    return (m_limbID == limbLabel.getLimbID() && 
      m_center == limbLabel.getCenter() && m_angle == limbLabel.getAngle());
  }

  bool LimbLabel::operator!=(const LimbLabel &limbLabel) const noexcept
  {
    return !(*this == limbLabel);
  }

  cv::Point2f LimbLabel::getCenter(void) const noexcept
  {
    return m_center;
  }

  std::vector<Score> LimbLabel::getScores(void) const noexcept
  {
    return m_scores;
  }

  void LimbLabel::setScores(std::vector <Score> scores) noexcept
  {
    m_scores = scores;
  }

  int LimbLabel::getLimbID(void) const noexcept
  {
    return m_limbID;
  }

  float LimbLabel::getAngle(void) const noexcept
  {
    return m_angle;
  }

  std::vector <cv::Point2f> LimbLabel::getPolygon(void) const noexcept
  {
    return m_polygon;
  }

  bool LimbLabel::getIsOccluded(void) const noexcept
  {
    return m_isOccluded;
  }

  bool LimbLabel::operator < (const LimbLabel &limbLabel) const noexcept
  {
    return getAvgScore(true) < limbLabel.getAvgScore(true); //true?
  }

  bool LimbLabel::operator > (const LimbLabel &limbLabel) const noexcept
  {
    return getAvgScore(true) > limbLabel.getAvgScore(true); //true?
  }

  float LimbLabel::getAvgScore(bool bNegativeToPositive) const noexcept
  {
    return getSumScore(bNegativeToPositive);
  }

  void LimbLabel::getEndpoints(cv::Point2f &p0, cv::Point2f &p1) const
  {
    const auto err = "Polygon should contain only 4 elements";
    if (m_polygon.size() != 4)
    {
      DebugMessage(err, 1);
      if (m_polygon.size() < 4)
        throw std::out_of_range(err);
      else
        throw std::logic_error(err);
    }
    //0-1 R, 1-2 G, 2-3 B, 3-0 P
    p0 = 0.5 * m_polygon.at(3) + 0.5 * m_polygon.at(0); //pink = parent
    p1 = 0.5 * m_polygon.at(1) + 0.5 * m_polygon.at(2); //green = child

    return; // this function needs to update the two endpoints passed into it
  }

  bool LimbLabel::containsPoint(cv::Point2f pt) const
  {
    //decide whether this point belongs to this polygon or not
    const auto err = "Polygon should contain only 4 elements";
    if (m_polygon.size() != 4)
    {
      DebugMessage(err, 1);
      if (m_polygon.size() < 4)
        throw std::out_of_range(err);
      else
        throw std::logic_error(err);
    }
    POSERECT <cv::Point2f> poserect(m_polygon.at(0), m_polygon.at(1), 
      m_polygon.at(2), m_polygon.at(3));

    return (poserect.containsPoint(pt) != -1);
  }

  void LimbLabel::addScore(Score detectionScore) noexcept
  {
    m_scores.push_back(detectionScore);
  }

  std::string LimbLabel::toString() const noexcept
  {
    auto retString = std::string("");
    retString += std::to_string(m_limbID) + " ";
    retString += std::to_string(m_center.x) + " " + 
      std::to_string(m_center.y) + " ";
    retString += std::to_string(m_angle) + " ";
    cv::Point2f c0, c1;
    getEndpoints(c0, c1);
    retString += std::to_string(c0.x) + " " + std::to_string(c0.y) + " ";
    retString += std::to_string(c1.x) + " " + std::to_string(c1.y) + " ";
    retString += std::to_string(m_isOccluded) + " ";
    for (const auto &i : m_scores)
    {
      retString += i.getDetName() + " ";
      retString += std::to_string(i.getScore()) + " ";
    }
    return retString;
  }

  float LimbLabel::getSumScore(bool bNegativeToPositive) const noexcept
  {
    //@TODO: fix if multiple scores are added
    auto sum = 0.0f;
    for_each(m_scores.begin(), m_scores.end(), [&](const auto &s)
    {
      sum += (bNegativeToPositive ? std::abs(s.getScore()) : s.getScore()) * s.getCoeff();
    });
    return sum;
  }

  void LimbLabel::Resize(float factor) noexcept
  {
    m_center.x *= factor;
    m_center.y *= factor;
    for (auto &p : m_polygon)
    {
      p.x *= factor;
      p.y *= factor;
    }
  }

}
