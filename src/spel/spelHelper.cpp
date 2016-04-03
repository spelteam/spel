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

  cv::Mat spelHelper::rotateImageToDefault(const cv::Mat &imgSource, const POSERECT <cv::Point2f> &initialRect, const float angle, const cv::Size &size)
  {
    auto partImage = cv::Mat(size, CV_8UC3, cv::Scalar(0, 0, 0));
    auto center = initialRect.GetCenter<cv::Point2f>();
    auto newCenter = cv::Point2f(0.5f * size.width, 0.5f * size.height);
    auto width = imgSource.size().width; // !!! For testing
    auto height = imgSource.size().height; // !!! For testing
    for (auto x = 0; x < size.width; x++)
    {
      for (auto y = 0; y < size.height; y++)
      {
        auto p = cv::Point2f(static_cast<float>(x), static_cast<float>(y));
        p = spelHelper::rotatePoint2D(p, newCenter, angle) + center - newCenter;
        try
        {
          if (0 <= p.x && 0 <= p.y && p.x < width - 1 && p.y < height - 1) // !!! For testing
            if (0 <= x && x < size.width - 1 && 0 <= y && y < size.height - 1) // !!! For testing
              partImage.at<cv::Vec3b>(y, x) = imgSource.at<cv::Vec3b>(static_cast<int>(std::roundf(p.y)), static_cast<int>(std::roundf(p.x)));
        }
        catch (...)
        {
          std::stringstream ss;
          ss << "Couldn't get value of indeces " << "[" << x << "][" << y << "] from indeces [" << p.x << "][" << p.y << "]";
          DebugMessage(ss.str(), 1);
          throw std::out_of_range(ss.str());
        }
      }
    }
    return partImage;
  }
  
  cv::Point2f spelHelper::round(const cv::Point2f& pt)
  {
    return cv::Point2f(std::roundf(pt.x), std::roundf(pt.y));
  }
}
