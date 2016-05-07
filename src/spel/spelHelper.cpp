#include "spelHelper.hpp"
#ifdef UNIX
#include <uuid/uuid.h>
#elif WINDOWS
#include <rpc.h>
#endif
#include <limits>
#include <fstream>
#include <cstdlib>
#include <random>
#include <ctime>
// windows defines max so we need to undefine this here
#ifdef max
#undef max
#endif

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
        variationCoeff = variationCoeff * (1 + 1 / (4 * scoreValues.size()));
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

  POSERECT<cv::Point2f> spelHelper::round(const POSERECT<cv::Point2f>& rect)
  {
    return POSERECT<cv::Point2f>(round(rect.point1), round(rect.point2),
      round(rect.point3), round(rect.point4));
  }

  std::string spelHelper::getRandomStr(void) noexcept
  {
    static std::default_random_engine dre(time(0));
    static std::uniform_int_distribution<int> di(0, std::numeric_limits<int>::max());
    return std::to_string(di(dre));
  }

  std::string spelHelper::getTempFileName(void)
  {
#ifdef WINDOWS
    char buf[MAX_PATH], strName[MAX_PATH];
    auto ret = GetTempPath(MAX_PATH, buf);
    if (ret > MAX_PATH || ret == 0)
    {
      std::stringstream ss;
      ss << "Can't get temporary directory";
      DebugMessage(ss.str(), 1);
      throw std::runtime_error(ss.str());
    }
    while (true)
    {
      std::string str(buf);
      if (str.back() != '\\')
        str.push_back('\\');
      str += getGUID();
      if (!checkFileExists(str))
        return str;
    }
#elif UNIX
    const auto env = getenv("TMPDIR");
    std::string tmp;
    if (env != 0)
      tmp = env;
    else
      tmp = "/tmp/";
    if (tmp.back() != '/')
      tmp.push_back('/');
    while (true)
    {
      auto str = tmp + getGUID();
      if (!checkFileExists(str))
        return str;
    }
#else
#error "Unsupported version of OS"
#endif
  }

  bool spelHelper::checkFileExists(std::string file) noexcept
  {
    std::ifstream f(file);
    return f.good();
  }

  std::string spelHelper::getGUID(void) noexcept
  {
    std::string guid;
#ifdef WINDOWS
    UUID uuid = { 0 };
    UuidCreate(&uuid);
    RPC_CSTR str = NULL;
    if (UuidToString(&uuid, &str) == RPC_S_OK)
    {
      guid = (char*)str;
      RpcStringFree(&str);
    }
    else
      guid = getRandomStr();
#elif UNIX
    uuid_t uuid;
    uuid_generate(uuid);
    char buf[37];
    uuid_unparse(uuid, buf);
    guid = buf;
#else
#error "Unsupported version of OS"
#endif
    std::transform(guid.begin(), guid.end(), guid.begin(), std::tolower);
    return guid;
  }
}
