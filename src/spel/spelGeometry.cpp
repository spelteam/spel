#include "spelGeometry.hpp"

namespace SPEL
{
  const uchar C_[3][3] = { { 7, 0, 1 },{ 6, 0, 2 },{ 5, 4, 3 } }; // Matrix for search contour operation
  const cv::Point2i P_[8] = { cv::Point2i(0, -1), cv::Point2i(1, -1), cv::Point2i(1, 0), cv::Point2i(1, 1), cv::Point2i(0, 1), cv::Point2i(-1, 1), cv::Point2i(-1, 0), cv::Point2i(-1, -1) }; // Matrix for search contour operation

  // Search contour iteration
  bool f_(cv::Mat mask, cv::Point2i &p0, cv::Point2i &p1, cv::Point2i p00)
  {
    int Q = 9;
    cv::Size size = mask.size();
    bool color = 0;
    cv::Point2i d = p0 - p1, p = p0;
    int n = C_[d.y + 1][d.x + 1];

    int  k = 0;
    while (!color)
    { 
      p0 = p;
      p = p1 + P_[n];
      if (k > 7) p = p00;
      if((p.x >= 0) && (p.x < size.width) && (p.y >= 0) && (p.y < size.height))
      { color = (mask.at<uchar>(p.y, p.x) > Q); }
      else { color = 0; }
      n++;
      if (n > 7) n = 0;
      k++;
    }
    p1 = p;
    return (p1 == p00);
  }

  // Search all ROI on the mask and return coordinates of max ROI: {topLeft, bottomRight}
  cv::Rect SearchROI_(cv::Mat mask)
  {
    cv::Size size = mask.size();
    int cols = size.width;
    int rows = size.height;

    int Q = 10 - 1;
    int maxArea = 0;
    int N = -1;

    std::vector<std::pair<cv::Point2i, cv::Point2i>> ROI;

    for (int y = 0; y < rows; y++)
      for (int x = 0; x < cols; x++)
      {
        cv::Point2i p(x, y);
        bool b = false;
        for (int i = 0; i < ROI.size(); i++)
        {
          if((p.x >= ROI[i].first.x) && (p.x <= ROI[i].second.x) && (p.y >= ROI[i].first.y) && (p.y <= ROI[i].second.y))
          {
            x = ROI[i].second.x;
            b = true;
          }
        }
        if ((!b) && (mask.at<uchar>(y, x) > Q))
        {
          cv::Point2i  p1 = p, p0 = p + cv::Point2i(-1, 0), p00 = p;

          bool FindedStartPoint = false;
          cv::Point2i A(cols, rows), B(0, 0);

          while (!FindedStartPoint)
          {
            FindedStartPoint = f_(mask, p0, p1, p00);

            if (p1.x < A.x) A.x = p1.x;
            if (p1.x > B.x) B.x = p1.x;
            if (p1.y < A.y) A.y = p1.y;
            if (p1.y > B.y) B.y = p1.y;
          }
          ROI.push_back(std::pair<cv::Point2f, cv::Point2f>(A, B));
          cv::Point2f P = B - A;
          float S = P.x*P.y;
          if ( S > maxArea)
          {
            maxArea = static_cast<int>(S);
            N = static_cast<int>(ROI.size()) - 1;
          }
        }		
      }
    std::vector<cv::Point2i> temp;
    if (N > -1)
    {
      temp.push_back(ROI[N].first);
      temp.push_back(ROI[N].second);
    }

    cv::Rect maskROI(temp[0], temp[1]);
    /*cv::Size borderSize = cv::Size(5, 5);
    borderSize += borderSize;
    maskROI = resizeROI_(maskROI, maskROI.size() + borderSize, mask.size());*/

    return maskROI;
  }

  cv::Rect resizeROI_(cv::Rect ROI, cv::Size NewROISize, cv::Size ImageSize)
  {
    float dx = static_cast<float>(NewROISize.width - ROI.width);
    float dy = static_cast<float>(NewROISize.height - ROI.height);
    float x = static_cast<float>(ROI.x);
    float y = static_cast<float>(ROI.y);

    cv::Point2f p0 = cv::Point2f(x, y) - 0.5f*cv::Point2f(dx, dy);

    if (p0.x < 0) p0.x = 0;
    if (p0.y < 0) p0.y = 0;

    if (ImageSize != cv::Size(0, 0))
    {
      if (static_cast<int>(p0.x) + NewROISize.width > ImageSize.width)
        NewROISize.width = ImageSize.width - static_cast<int>(p0.x);
      if (static_cast<int>(p0.y) + NewROISize.height > ImageSize.height)
        NewROISize.height = ImageSize.height - static_cast<int>(p0.y);
    }

    return cv::Rect(p0, NewROISize);
  }
}