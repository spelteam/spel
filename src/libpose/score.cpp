#include "score.hpp"

Score::Score(void)
{
  score = 0;
  detName = "";
}

Score::Score(float sc, string name)
{
  score = sc;
  detName = name;
}

Score & Score::operator=(const Score &s)
{
  detName = s.getDetName();
  score = s.getScore();
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

