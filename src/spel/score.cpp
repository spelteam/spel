#include "score.hpp"
// See Score.hpp for more info
namespace SPEL
{
  Score::Score(void) noexcept
  {
    m_score = 0;
    m_detName = "";
    m_coeff = 1.0f;
    m_isWeak = false;
  }

  Score::Score(const float score, const std::string & name) noexcept
  {
    m_score = score;
    m_detName = name;
    m_coeff = 1.0f;
    m_isWeak = false;
  }

  Score::Score(const float score, const std::string &name, const float coeff)
    noexcept
  {
    m_score = score;
    m_detName = name;
    m_coeff = coeff;
    m_isWeak = false;
  }

  Score::Score(const float score, const std::string & name, const float coeff,
    const bool isWeak) noexcept
  {
    m_score = score;
    m_detName = name;
    m_coeff = coeff;
    m_isWeak = isWeak;
  }

  Score::Score(const Score & score) noexcept
    : m_score(score.m_score),
    m_detName(score.m_detName),
    m_coeff(score.m_coeff),
    m_isWeak(score.m_isWeak)
  {
  }

  Score::Score(Score && score) noexcept
    : m_score(std::move(score.m_score)),
    m_detName(std::move(score.m_detName)),
    m_coeff(std::move(score.m_coeff)),
    m_isWeak(std::move(score.m_isWeak))
  {
  }

  Score::~Score(void) noexcept
  {
  }

  Score &Score::operator=(const Score &score) noexcept
  {
    if (this == &score)
      return *this;
    m_detName = score.m_detName;
    m_score = score.m_score;
    m_coeff = score.m_coeff;
    m_isWeak = score.m_isWeak;
    return *this;
  }

  float Score::getScore(void) const noexcept
  {
    return m_score;
  }

  void Score::setScore(const float score) noexcept
  {
    m_score = score;
  }

  std::string Score::getDetName(void) const noexcept
  {
    return m_detName;
  }

  void Score::setDetName(const std::string &detName) noexcept
  {
    m_detName = detName;
  }

  bool Score::operator<(const Score &score) const noexcept
  {
    return (m_score * m_coeff < score.m_score * score.m_coeff);
  }

  bool Score::operator>(const Score &score) const noexcept
  {
    return (m_score * m_coeff > score.m_score * score.m_coeff);
  }

  bool Score::operator==(const Score &score) const noexcept
  {
    return (m_score * m_coeff == score.m_score * score.m_coeff && 
      m_detName == score.m_detName);
  }

  bool Score::operator!=(const Score &score) const noexcept
  {
    return !(*this == score);
  }

  Score & Score::operator=(Score && score) noexcept
  {
    m_score = std::move(score.m_score);
    std::swap(m_detName, score.m_detName);
    m_coeff = std::move(score.m_coeff);
    m_isWeak = std::move(score.m_isWeak);

    return *this;
  }

  float Score::getCoeff(void) const noexcept
  {
    return m_coeff;
  }

  void Score::setCoeff(const float coeff) noexcept
  {
    m_coeff = coeff;
  }

  bool Score::getIsWeak(void) const noexcept
  {
    return m_isWeak;
  }

  void Score::setIsWeak(const bool isWeak) noexcept
  {
    m_isWeak = isWeak;
  }

}
