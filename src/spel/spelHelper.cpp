#include "spelHelper.hpp"

namespace SPEL
{
  void spelHelper::RecalculateScoreIsWeak(std::vector <LimbLabel> &labels, std::string detectorName, float standardDiviationTreshold)
  {
    //@TODO: Ignore this function for now, will be modified before release
    std::vector <float> scoreValues;
    float min = 1.0f;
    const uint32_t minCount = 600;
    float sum = 0;
    for (std::vector <LimbLabel>::iterator i = labels.begin(); i != labels.end(); ++i)
    {
      std::vector <Score> scores = i->getScores();
      for (std::vector <Score>::iterator j = scores.begin(); j != scores.end(); ++j)
      {
        if (j->getDetName() == detectorName && j->getScore() > 0)
        {
          min = min > j->getScore() ? j->getScore() : min;
          scoreValues.push_back(j->getScore());
          sum += j->getScore();
        }
      }
    }
    bool isWeak = true;
    if (scoreValues.size() > 0)
    {
      float mean = (float)sum / (float)scoreValues.size();
      float sqrSum = 0;
      for (std::vector <float>::iterator i = scoreValues.begin(); i != scoreValues.end(); ++i)
      {
        sqrSum += pow((*i) - mean, 2);
      }
      float variance = (float)sqrSum / (float)(scoreValues.size());
      float standardDeviation = sqrt(variance);
      float variationCoeff = standardDeviation / (mean - min);
      float dispersionIndex = variance / (mean - min);
      if (scoreValues.size() < minCount)
        variationCoeff = variationCoeff * (1.0 + 1 / (4 * scoreValues.size()));
      isWeak = dispersionIndex < standardDiviationTreshold;
    }
    for (std::vector <LimbLabel>::iterator i = labels.begin(); i != labels.end(); ++i)
    {
      std::vector <Score> scores = i->getScores();
      for (std::vector <Score>::iterator j = scores.begin(); j != scores.end(); ++j)
      {
        if (j->getDetName() == detectorName)
        {
          j->setIsWeak(isWeak);
        }
      }
      i->setScores(scores);
    }
  }

}
