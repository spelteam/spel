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

  void PoseHelper::RecalculateScoreIsWeak(vector <LimbLabel> &labels, string detectorName, float varianceTreshold)
  {
    vector <float> scoreValues;
    float avg = 0;
    for (vector <LimbLabel>::iterator i = labels.begin(); i != labels.end(); ++i)
    {
      vector <Score> scores = i->getScores();
      for (vector <Score>::iterator j = scores.begin(); j != scores.end(); ++j)
      {
        if (j->getDetName() == detectorName)
        {
          scoreValues.push_back(j->getScore());
          avg += j->getScore();
        }
      }
    }
    avg /= scoreValues.size();
    float sum = 0;
    for (vector <float>::iterator i = scoreValues.begin(); i != scoreValues.end(); ++i)
    {
      sum += pow((*i) - avg, 2);
    }
    float variance = sqrt(sum / (scoreValues.size() - 1));
    for (vector <LimbLabel>::iterator i = labels.begin(); i != labels.end(); ++i)
    {
      vector <Score> scores = i->getScores();
      for (vector <Score>::iterator j = scores.begin(); j != scores.end(); ++j)
      {
        if (j->getDetName() == detectorName)
        {
          j->setIsWeak(variance < varianceTreshold);
        }
      }
      i->setScores(scores);
    }
  }

}
