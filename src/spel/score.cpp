// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
#include "score.hpp"
#include "spelHelper.hpp"
// See Score.hpp for more info
namespace SPEL
{
  Score::Score(void) : Score(0, std::string(""), 1.0f, false)
  {
  }

  Score::Score(const float score, const std::string & name) 
    : Score(score, name, 1.0f, false)
  {
  }

  Score::Score(const float score, const std::string &name, const float coeff)
    : Score(score, name, coeff, false)
  {
  }

  Score::Score(const float score, const std::string & name, const float coeff,
    const bool isWeak) 
    : m_score(score), m_detName(name), m_coeff(coeff), m_isWeak (isWeak)    
  {
  }

  Score::Score(const Score & score) 
    : m_score(score.m_score),
    m_detName(score.m_detName),
    m_coeff(score.m_coeff),
    m_isWeak(score.m_isWeak)
  {
  }

  Score::Score(Score && score) 
    : m_score(std::move(score.m_score)),
    m_detName(std::move(score.m_detName)),
    m_coeff(std::move(score.m_coeff)),
    m_isWeak(std::move(score.m_isWeak))
  {
  }

  Score::~Score(void) 
  {
  }

  Score &Score::operator=(const Score &score) 
  {
    if (this == &score)
      return *this;
    m_detName = score.m_detName;
    m_score = score.m_score;
    m_coeff = score.m_coeff;
    m_isWeak = score.m_isWeak;
    return *this;
  }

  float Score::getScore(void) const 
  {
    return m_score;
  }

  void Score::setScore(const float score) 
  {
    m_score = score;
  }

  std::string Score::getDetName(void) const 
  {
    return m_detName;
  }

  void Score::setDetName(const std::string &detName) 
  {
    m_detName = detName;
  }

  bool Score::operator<(const Score &score) const 
  {
    return spelHelper::compareFloat(m_score * m_coeff, 
      score.m_score * score.m_coeff) < 0;
  }

  bool Score::operator>(const Score &score) const 
  {
    return spelHelper::compareFloat(m_score * m_coeff, 
      score.m_score * score.m_coeff) > 0;
  }

  bool Score::operator==(const Score &score) const 
  {
    return (spelHelper::compareFloat(m_score * m_coeff, 
      score.m_score * score.m_coeff) == 0 &&
      m_detName == score.m_detName);
  }

  bool Score::operator!=(const Score &score) const 
  {
    return !(*this == score);
  }

  Score & Score::operator=(Score && score) 
  {
    m_score = std::move(score.m_score);
    std::swap(m_detName, score.m_detName);
    m_coeff = std::move(score.m_coeff);
    m_isWeak = std::move(score.m_isWeak);

    return *this;
  }

  float Score::getCoeff(void) const 
  {
    return m_coeff;
  }

  void Score::setCoeff(const float coeff) 
  {
    m_coeff = coeff;
  }

  bool Score::getIsWeak(void) const 
  {
    return m_isWeak;
  }

  void Score::setIsWeak(const bool isWeak) 
  {
    m_isWeak = isWeak;
  }

}
