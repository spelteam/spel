#include "score.hpp"

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

string Score::getDetName(void) const
{
  return detName;
}

