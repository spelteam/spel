// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
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

namespace SPEL
{
  void spelHelper::RecalculateScoreIsWeak(std::vector <LimbLabel> &labels,
    const std::string &detectorName, const float standardDiviationTreshold)

  {
    //@TODO: Ignore this function for now, will be modified before release
    std::vector <float> scoreValues;
    auto min = 1.0f;
    const auto minCount = 600;
    auto sum = 0.0f;
    for (const auto &i : labels)
    {
      const auto &scores = i.getScores();
      for (const auto &j : scores)
      {
        const auto val = j.getScore();
        if (j.getDetName() == detectorName && val > 0.0f)
        {
          if (min > val)
            min = val;
          scoreValues.push_back(val);
          sum += val;
        }
      }
    }
    auto isWeak = true;
    if (scoreValues.size() > 0)
    {
      const auto mean = sum / static_cast<float>(scoreValues.size());
      auto sqrSum = 0.0f;
      for (const auto &i : scoreValues)
        sqrSum += pow(i - mean, 2);
      const auto variance = sqrSum / static_cast<float>(scoreValues.size());
      auto variationCoeff = sqrt(variance) / (mean - min);
      if (scoreValues.size() < minCount)
        variationCoeff *= (1 + 1 / (4 * scoreValues.size()));
      isWeak = (variance / (mean - min)) < standardDiviationTreshold;
    }
    for (auto &i : labels)
    {
      auto scores = i.getScores();
      for (auto &j : scores)
        if (j.getDetName() == detectorName)
          j.setIsWeak(isWeak);
      i.setScores(scores);
    }
  }

  cv::Mat spelHelper::rotateImageToDefault(const cv::Mat &imgSource,
    const spelRECT <cv::Point2f> &initialRect, const float angle,
    const cv::Size &size)
  {
    auto partImage = cv::Mat(size, CV_8UC3, cv::Scalar(0, 0, 0));
    const auto &center = initialRect.GetCenter<cv::Point2f>();
    const auto &newCenter = cv::Point2f(0.5f * size.width, 0.5f * size.height);
    const auto width = imgSource.size().width; // !!! For testing
    const auto height = imgSource.size().height; // !!! For testing
    for (auto x = 0; x < size.width; ++x)
    {
      for (auto y = 0; y < size.height; ++y)
      {
        auto p = cv::Point2f(static_cast<float>(x), static_cast<float>(y));
        p = spelHelper::rotatePoint2D(p, newCenter, angle) + center - newCenter;
        try
        {
          if (0.0f <= p.x && 0.0f <= p.y && p.x < width - 1.0f &&
            p.y < height - 1.0f) // !!! For testing
            if (0.0f <= x && x < size.width - 1.0f && 0.0f <= y &&
              y < size.height - 1.0f) // !!! For testing
              partImage.at<cv::Vec3b>(y, x) = imgSource.at<cv::Vec3b>(
                static_cast<int>(std::round(p.y)),
                static_cast<int>(std::round(p.x)));
        }
        catch (...)
        {
          std::stringstream ss;
          ss << "Couldn't get value of indeces " << "[" << x << "][" << y <<
            "] from indeces [" << p.x << "][" << p.y << "]";
          DebugMessage(ss.str(), 1);
          throw std::out_of_range(ss.str());
        }
      }
    }
    return partImage;
  }

  cv::Point2f spelHelper::round(const cv::Point2f& pt)
  {
    return cv::Point2f(std::round(pt.x), std::round(pt.y));
  }

  spelRECT<cv::Point2f> spelHelper::round(const spelRECT<cv::Point2f>& rect)
  {
    return spelRECT<cv::Point2f>(round(rect.point1), round(rect.point2),
      round(rect.point3), round(rect.point4));
  }

  std::string spelHelper::getRandomStr(void)
  {
    static std::default_random_engine dre(static_cast<unsigned int>(time(0)));
    static std::uniform_int_distribution<int> di(0, std::numeric_limits<int>::max());
    return std::to_string(di(dre));
  }

  std::string spelHelper::getTempFileName(const std::string &extension)
  {
    auto ext = extension;
    if (ext.find('.') == std::string::npos)
      ext.insert(0, 1, '.');
#ifdef WINDOWS
    char buf[MAX_PATH];
    const auto ret = GetTempPath(MAX_PATH, buf);
    if (ret > MAX_PATH || ret == 0)
    {
      const auto &str = std::string("Can't get temporary directory");
      DebugMessage(str, 1);
      throw std::runtime_error(str);
    }
    while (true)
    {
      std::string str(buf);
      if (str.back() != '\\')
        str.push_back('\\');
      str += getGUID();
      if (!ext.empty())
        str += ext;
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
      if (!ext.empty())
        str += ext;
      if (!checkFileExists(str))
        return str;
    }
#else
#error "Unsupported version of OS"
#endif
  }

  bool spelHelper::checkFileExists(const std::string &file)
  {
    std::ifstream f(file);
    return f.good();
  }

  void spelHelper::copyFile(const std::string &dst, const std::string &src)
  {
    std::ifstream ifs(src, std::ios::binary);
    std::ofstream ofs(dst, std::ios::binary);
    if (ifs.bad() || ofs.bad())
    {
      std::stringstream ss;
      ss << "Can't copy the content of the file '" << src <<
        "' to the file '" << dst << "'";
      DebugMessage(ss.str(), 1);
      throw std::out_of_range(ss.str());
    }
    ofs << ifs.rdbuf();
    ifs.close();
    ofs.close();
  }

  float spelHelper::getAngle(const cv::Point2f & j0, const cv::Point2f & j1)
  {
    return spelHelper::angle2D(1.0f, 0.0f, j1.x - j0.x, j1.y - j0.y) *
      (180.0f / static_cast<float>(M_PI));
  }

  float spelHelper::getAngle(const cv::Point2f & point)
  {
    return spelHelper::angle2D(1.0f, 0.0f, point.x, point.y) *
      (180.0f / static_cast<float>(M_PI));
  }

  float spelHelper::getAngle(const BodyJoint & j0, const BodyJoint & j1)
  {
    return spelHelper::getAngle(j0.getImageLocation(), j1.getImageLocation());
  }

  void spelHelper::mergeParameters(std::map<std::string, float>& dst, const std::map<std::string, float>& src)
  {
    if (!src.empty())
      dst.insert(src.begin(), src.end());
  }

  long spelHelper::clock_to_ms(long t)
  {
    return t * 1000 / CLOCKS_PER_SEC;
  }

  std::string spelHelper::getFileExt(const std::string & path, 
    const std::string &defaultExt)
  {
    const auto pos = path.rfind('.');
    if (pos != std::string::npos)
    {
      auto text = path.substr(pos);
      if (!text.empty())
        return text;
    }
    return defaultExt;
  }

  std::string spelHelper::getTempFileCopy(const std::string & src, const std::string & defaultExt)
  {
    const auto dst = spelHelper::getTempFileName(spelHelper::getFileExt(src, defaultExt));
    spelHelper::copyFile(dst, src);

    return dst;
  }

  std::string spelHelper::getGUID(void)
  {
    std::string guid;
#ifdef WINDOWS
    UUID uuid = {};
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
    std::transform(guid.begin(), guid.end(), guid.begin(), ::tolower);
    return guid;
  }
}
