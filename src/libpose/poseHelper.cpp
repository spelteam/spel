#include "poseHelper.hpp"

namespace SPEL
{

  double PoseHelper::angle2D(double x1, double y1, double x2, double y2)
  {
    //input has a zero vector
    if ((x1 == 0.0 && y1 == 0.0) || (x2 == 0.0 && y2 == 0.0)){
      //zero vector is both parallel and perpendicular to every vector
      return 0.0;
    }

    double dtheta, theta1, theta2;
    //angle between Ox and first vector
    theta1 = atan2(y1, x1);
    //angle between Ox and second vector
    theta2 = atan2(y2, x2);
    //angle between first and second vector
    dtheta = theta2 - theta1;
    //normalize angle to range [-PI;PI]
    while (dtheta > M_PI)
      dtheta -= (M_PI * 2.0);
    while (dtheta < -M_PI)
      dtheta += (M_PI * 2.0);
    return(dtheta);
  }

  double PoseHelper::interpolateFloat(double prevAngle, double nextAngle, int step, int numSteps)
  {
    double t;
    if (numSteps != 0)
      t = (double)step / (double)numSteps;
    else
      t = 0;
    return prevAngle*(1 - t) + nextAngle*t;
  }

  void PoseHelper::RecalculateScoreIsWeak(vector <LimbLabel> &labels, string detectorName, float standardDiviationTreshold)
  {
    vector <float> scoreValues;
    float avg = 0;
    float min = 1.0f;
    for (vector <LimbLabel>::iterator i = labels.begin(); i != labels.end(); ++i)
    {
      vector <Score> scores = i->getScores();
      for (vector <Score>::iterator j = scores.begin(); j != scores.end(); ++j)
      {
        if (j->getDetName() == detectorName && j->getScore() > 0)
        {
          min = min > j->getScore() ? j->getScore() : min;
          scoreValues.push_back(j->getScore());
          avg += j->getScore();
        }
      }
    }
    avg = (avg - min) / scoreValues.size();
    float sum = 0;
    for (vector <float>::iterator i = scoreValues.begin(); i != scoreValues.end(); ++i)
    {
      sum += pow((*i) - avg, 2);
    }
    float standardDiviation = sqrt(sum / (scoreValues.size() - 1)) / avg;
    bool isWeak = standardDiviation < standardDiviationTreshold;
    for (vector <LimbLabel>::iterator i = labels.begin(); i != labels.end(); ++i)
    {
      vector <Score> scores = i->getScores();
      for (vector <Score>::iterator j = scores.begin(); j != scores.end(); ++j)
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
