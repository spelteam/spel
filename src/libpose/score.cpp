#include "score.hpp"
// See Score.hpp for more info

// default constructor
Score::Score(void)
{
  score = 0;
  detName = "";
}

// constructor with params
Score::Score(float sc, string name)
{
  score = sc;
  detName = name;
}

Score &Score::operator=(const Score &s)
{
  if (this == &s)
  {
    return *this;
  }
  this->detName = s.getDetName();
  this->score = s.getScore();
  return *this;
}

float Score::getScore(void) const
{
  return score;
}

void Score::setScore(float _score)
{
  score = _score;
}

string Score::getDetName(void) const
{
  return detName;
}

void Score::setDetName(string _detName)
{
  detName = _detName;
}

bool Score::operator<(const Score &s) const
{
  return (this->score < s.getScore());
}

bool Score::operator>(const Score &s) const
{
  return (this->score > s.getScore());
}

bool Score::operator==(const Score &s) const
{
  return (this->score == s.getScore());
}

bool Score::operator!=(const Score &s) const
{
  return !(*this == s);
}

