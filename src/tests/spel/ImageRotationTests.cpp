// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include "imagehogsimilaritymatrix.hpp"
#include "lockframe.hpp"
#include "TestsFunctions.hpp"
#include "spelHelper.hpp"
#include <time.h>

using namespace std;
using namespace cv;

namespace SPEL
{
// FUNCTIONS

  //  Part polygon points position:
  //  p3--j0--p0
  //  |        |
  //  |        |
  //  |        |
  //  |        |
  //  p4--j1--p1
    
  // New ROI polygon dimension = (NewSize.width, NewSizeheight) 
  std::vector<cv::Point2f> ResizeSlantedROI(std::vector<cv::Point2f> &ROIRect, const cv::Size &NewSize)
  {
    cv::Point2f dW = ROIRect[3] - ROIRect[0];
    float width = sqrt(dW.x*dW.x + dW.y*dW.y);
    cv::Point2f dH = ROIRect[1] - ROIRect[0];
    float height = sqrt(dH.x*dH.x + dH.y*dH.y); // polygon length

    float k_W = NewSize.width / width - 1.0f;
    float k_H = NewSize.height / height -1.0f;

    vector<Point2f> NewROIRect = ROIRect;
    NewROIRect[0] = NewROIRect[0] - 0.5*(k_W*dW + k_H*dH);
    NewROIRect[1] = NewROIRect[1] - 0.5*(k_W*dW - k_H*dH);
    NewROIRect[2] = NewROIRect[2] + 0.5*(k_W*dW + k_H*dH);
    NewROIRect[3] = NewROIRect[3] + 0.5*(k_W*dW - k_H*dH);

    return NewROIRect;
  }

  // New ROI polygon dimension = (polygon.width + 2*deltaWidth, polygon.height + 2*deltaHeight) 
  std::vector<cv::Point2f> ExtendSlantedROI(std::vector<cv::Point2f> &ROIRect, float deltaWidth, float deltaHeight)
  {
    cv::Point2f dW = ROIRect[3] - ROIRect[0];
    float width = sqrt(dW.x*dW.x + dW.y*dW.y);
    cv::Point2f dH = ROIRect[1] - ROIRect[0];
    float height = sqrt(dH.x*dH.x + dH.y*dH.y); // polygon length

    float k_W = deltaWidth / width;
    float k_H = deltaHeight / height;

    vector<Point2f> NewROIRect = ROIRect;
    NewROIRect[0] = NewROIRect[0] - (k_W*dW + k_H*dH);
    NewROIRect[1] = NewROIRect[1] - (k_W*dW - k_H*dH);
    NewROIRect[2] = NewROIRect[2] + (k_W*dW + k_H*dH);
    NewROIRect[3] = NewROIRect[3] + (k_W*dW - k_H*dH);

    return NewROIRect;
  }

  //spelHelper::RotateImageToDefault
  cv::Mat RotateImageToDefault(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
  {
    Point2d l = ROIRect[1] - ROIRect[0];
    Point2d d = ROIRect[3] - ROIRect[0];
    float angle = static_cast<float>(180.0*atan(d.y/d.x)/ M_PI);

    float width = static_cast<float>(sqrt(d.x*d.x + d.y*d.y));
    float height = static_cast<float>(sqrt(l.x*l.x + l.y*l.y));
    Size size(round(width), round(height));

    POSERECT<Point2f> rect(ROIRect[0], ROIRect[1], ROIRect[2], ROIRect[3]);

    return spelHelper::rotateImageToDefault(imgSource, rect, angle, size);
  }

  // Simplified cordinate system rotation
  cv::Mat DeRotate_0(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
  {
    cv::Point2f dw = ROIRect[3] - ROIRect[0];
    float width = sqrt(dw.x*dw.x + dw.y*dw.y);
    cv::Point2f dh = ROIRect[1] - ROIRect[0];
    float height = sqrt(dh.x*dh.x + dh.y*dh.y);

    int n = round(height), m = round(width);
    cv::Mat Image = cv::Mat(n + 1, m + 1, CV_8UC3, cv::Scalar(0, 0, 0));

    dw = dw*(1.0 / width); //static_cast<float>(m));
    dh = dh*(1.0 / height); //static_cast<float>(n));

    cv::Point2f currentPoint;
    for (int i = 0; i < n; i++)
      for (int k = 0; k < m; k++)
      {
        currentPoint = ROIRect[0] + static_cast<float>(i)*dh + static_cast<float>(k)*dw;
        if (currentPoint.x >= 0 && currentPoint.x < imgSource.cols && currentPoint.y >= 0 && currentPoint.y < imgSource.rows)
          Image.at<cv::Vec3b>(i, k) = imgSource.at<cv::Vec3b>(int(std::roundf(currentPoint.y)), int(std::roundf(currentPoint.x)));
      }

    return Image;
  }

  // Scanning the  vertices of a square  around the pixel center
  cv::Mat DeRotate_7(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
  {
    cv::Point2d dw = ROIRect[3] - ROIRect[0];
    double width = sqrt(dw.x*dw.x + dw.y*dw.y);
    cv::Point2d dh = ROIRect[1] - ROIRect[0];
    double height = sqrt(dh.x*dh.x + dh.y*dh.y);

    int n = std::round(height) , m = std::round(width) ;
    cv::Mat Image = cv::Mat(n+1, m+1, CV_8UC3, cv::Scalar(0, 0, 0));

    dw = dw*(1.0 / width);  /*static_cast<double>(m));*/
    dh = dh*(1.0 / height); /*static_cast<double>(n));*/

    cv::Point2d p0(ROIRect[0]);
    cv::Point2d currentPoint(ROIRect[0]);

    for (int i = 0; i <= n; i++)
      for (int k = 0; k <= m; k++)
      {
        std::vector<cv::Vec3b> colors;
        std::vector<int> counts;
        for (float p = -1.0f; p <= 1.0f; p++)
          for (float t = -1.0f; t <= 1.0f; t++)
          {
            currentPoint = p0 + static_cast<float>(i)*dh + static_cast<float>(k)*dw  + 0.3*cv::Point2d(p, t); /*p*0.2*dh + t*0.2*dw)*/
            cv::Vec3b color = imgSource.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x)));
            int Q = -1;
            for (int q = 0; q < colors.size(); q++)
              if(colors[q] == color) 
              {
                counts[q]++;
                Q = q;
              }
            if (Q == -1)
            {
              colors.push_back(color);
              counts.push_back(1);
            }
          }
          int max = 0;
          cv::Vec3b color(0, 0, 0);
          for (int t = 0; t < colors.size(); t++)
            if(counts[t] > max)
            {
              max = counts[t];
              color = colors[t];
            }
          Image.at<cv::Vec3b>(i, k) = color;
          colors.clear();
          counts.clear();
      }

    return Image;
  }

  // Rotation using mixed color
  cv::Mat DeRotate_7_1(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
  {
    cv::Point2f dw = ROIRect[3] - ROIRect[0];
    float width = sqrt(dw.x*dw.x + dw.y*dw.y);
    cv::Point2f dh = ROIRect[1] - ROIRect[0];
    float height = sqrt(dh.x*dh.x + dh.y*dh.y);

    int n = static_cast<int>(round(height)), m = static_cast<int>(round(width));

    cv::Mat Result(n + 1, m + 1, CV_8UC3, cv::Scalar(0, 0, 0));

    dw = dw*(1.0f / width);
    dh = dh*(1.0f / height);

    cv::Point2f p0(ROIRect[0].x, ROIRect[0].y);
    cv::Point2f currentPoint(p0);
      
    float N = 10.0f;
    for (int i = 0; i < n; i++)
      for (int k = 0; k < m; k++)
      {
        std::vector<cv::Vec3b> colors;
        std::vector<int> counts;

        Vec3f colorsSum(0.0f, 0.0f, 0.0f);
        int count = 0;
        for (float p = 0; p < N; p++)
          for (float t = 0; t < N; t++)
          {
            currentPoint = p0 + static_cast<float>(i)*dh + static_cast<float>(k)*dw  /*- 0.5f*(dh + dw)*/ + cv::Point2f(p/N, t/N);
            if (currentPoint.x >= 0 && currentPoint.x <imgSource.cols && currentPoint.y >= 0 && currentPoint.y < imgSource.rows)
            {
              colorsSum += imgSource.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x)));			
              count++;
            }
            Result.at<cv::Vec3b>(i, k) = colorsSum / count;
          }
       }

    return Result;
  }



  // Simplified cordinate system rotation with visualization
  cv::Mat DeRotate_2_visualization(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
  {
    float s = 11.0f;
    auto ScalledImage = cv::Mat(imgSource.size().width*s, imgSource.size().height*s, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int x = 0; x < imgSource.size().width; x++)
      for (int y = 0; y < imgSource.size().height; y++)
      {
        for (int i = 0; i < s; i++)
          for (int k = 0; k < s; k++)
            ScalledImage.at<cv::Vec3b>(y*s+i, x*s+k) = imgSource.at<cv::Vec3b>(y,x);
      }

    cv::Point2f dw = ROIRect[3] - ROIRect[0];
    float width = sqrt(dw.x*dw.x + dw.y*dw.y);
    cv::Point2f dh = ROIRect[1] - ROIRect[0];
    float height = sqrt(dh.x*dh.x + dh.y*dh.y);

    int n = static_cast<int>(round(height)), m = static_cast<int>(round(width));
    cv::Mat Image = cv::Mat(n+1, m+1, CV_8UC3, cv::Scalar(0, 0, 0));


    dw = dw*(1.0f/ width);
    dh = dh*(1.0f/ height);

    cv::Point2f p0(round(ROIRect[0].x), round(ROIRect[0].y));
    cv::Point2f currentPoint(p0);

    for (int i = 0; i <= n; i++)
      for (int k = 0; k <= m; k++)	  
      {
        currentPoint = s*(p0 + static_cast<float>(i)*dh + static_cast<float>(k)*dw + 0.5f*(dh+dw));
        Image.at<cv::Vec3b>(i, k) = ScalledImage.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x)));
        ScalledImage.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x ))) *= 0.2f;
        ScalledImage.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x))) += cv:: Vec3b(100, 100, 100);
      }

    imwrite("DeRotate_2_ScannedPoints.bmp", ScalledImage);
    ScalledImage.release();

    return Image;
  }

  // Scanning vertices of a square  around the pixel center with visualization
  cv::Mat DeRotate_3_visualization(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
  {
    float s = 11.0f;
    auto ScalledImage = cv::Mat(imgSource.size().width*s, imgSource.size().height*s, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int x = 0; x < imgSource.size().width; x++)
      for (int y = 0; y < imgSource.size().height; y++)
        for (int i = 0; i < s; i++)
          for (int k = 0; k < s; k++)
            ScalledImage.at<cv::Vec3b>(y*s + i, x*s + k) = imgSource.at<cv::Vec3b>(y, x);

      cv::Point2f dw = ROIRect[3] - ROIRect[0];
      float width = sqrt(dw.x*dw.x + dw.y*dw.y);
      cv::Point2f dh = ROIRect[1] - ROIRect[0];
      float height = sqrt(dh.x*dh.x + dh.y*dh.y);

      int n = static_cast<int>(round(height)), m = static_cast<int>(round(width));
      cv::Mat Image = cv::Mat(n + 1, m + 1, CV_8UC3, cv::Scalar(0, 0, 0));

      dw = dw*(1.0f / static_cast<float>(m));
      dh = dh*(1.0f / static_cast<float>(n));

      cv::Point2f p0(round(ROIRect[0].x), round(ROIRect[0].y));
      cv::Point2f currentPoint(p0);
      
      for (int i = 0; i <= n; i++)
          for (int k = 0; k <= m; k++)
          {
            std::vector<cv::Vec3b> colors;
            std::vector<int> counts;
            for (float p = -1.0f; p <= 1.0f; p++)
              for (float t = -1.0f; t <= 1.0f; t++)
              {
                currentPoint = s*(p0 + static_cast<float>(i)*dh +0.5f*dh + 0.5f*dw + static_cast<float>(k)*dw + 0.3f*cv::Point2f(p, t)/*p*0.2f*dh + t*0.2f*dw*/);
                cv::Vec3b color = ScalledImage.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x)));
                ScalledImage.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x))) *= 0.2f;
                ScalledImage.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x))) += cv::Vec3b(100, 100, 100);
                int Q = -1;
                for (int q = 0; q < colors.size(); q++)
                {
                  if(colors[q] == color) 
                  {
                    counts[q]++;
                    Q = q;
                  }
                }
                if (Q == -1)
                {
                  colors.push_back(color);
                  counts.push_back(1);
                }
              }
              int max = 0;
              cv::Vec3b color(0, 0, 0);
              for (int t = 0; t < colors.size(); t++)
              {
                if(counts[t] > max)
                {
                  max = counts[t];
                  color = colors[t];
                }
              }
              Image.at<cv::Vec3b>(i, k) = color;
              colors.clear();
              counts.clear();
          }

    imwrite("DeRotate_3_ScannedPoints.bmp", ScalledImage);
    ScalledImage.release();
    //imwrite("DeRotate_3_Source.bmp", imgSource);
    return Image;
  }

 // Scanning vertices of a rhomb around the pixel center with visualization
  cv::Mat DeRotate_4_visualization(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
  {
    float s = 11.0f;
    auto ScalledImage = cv::Mat(imgSource.size().width*s, imgSource.size().height*s, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int x = 0; x < imgSource.size().width; x++)
      for (int y = 0; y < imgSource.size().height; y++)
        for (int i = 0; i < s; i++)
          for (int k = 0; k < s; k++)
            ScalledImage.at<cv::Vec3b>(y*s + i, x*s + k) = imgSource.at<cv::Vec3b>(y, x);

      cv::Point2f dw = ROIRect[3] - ROIRect[0];
      float width = sqrt(dw.x*dw.x + dw.y*dw.y);
      cv::Point2f dh = ROIRect[1] - ROIRect[0];
      float height = sqrt(dh.x*dh.x + dh.y*dh.y);

      int n = static_cast<int>(round(height)), m = static_cast<int>(round(width));
      cv::Mat Image = cv::Mat(n + 1, m + 1, CV_8UC3, cv::Scalar(0, 0, 0));

      dw = dw*(1.0f / static_cast<float>(m));
      dh = dh*(1.0f / static_cast<float>(n));

      cv::Point2f p0(round(ROIRect[0].x), round(ROIRect[0].y));
      cv::Point2f currentPoint(p0);
      
      for (int i = 0; i <= n; i++)
          for (int k = 0; k <= m; k++)
          {
            std::vector<cv::Vec3b> colors;
            std::vector<int> counts;
            for (float p = -1.0f; p <= 1.0f; p++)
              for (float t = -1.0f; t <= 1.0f; t++)
              {
                currentPoint = s*(p0 + static_cast<float>(i)*dh +0.5f*dh + 0.5f*dw + static_cast<float>(k)*dw + 0.2f*(p*dh + t*dw));
                cv::Vec3b color = ScalledImage.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x)));
                ScalledImage.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x))) *= 0.2f;
                ScalledImage.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x))) += cv::Vec3b(100, 100, 100);
                int Q = -1;
                for (int q = 0; q < colors.size(); q++)
                {
                  if(colors[q] == color) 
                  {
                    counts[q]++;
                    Q = q;
                  }
                }
                if (Q == -1)
                {
                  colors.push_back(color);
                  counts.push_back(1);
                }
              }
              int max = 0;
              cv::Vec3b color(0, 0, 0);
              for (int t = 0; t < colors.size(); t++)
              {
                if(counts[t] > max)
                {
                  max = counts[t];
                  color = colors[t];
                }
              }
              Image.at<cv::Vec3b>(i, k) = color;
              colors.clear();
              counts.clear();
          }

      imwrite("DeRotate_3_ScannedPoints.bmp", ScalledImage);
      ScalledImage.release();
      //imwrite("DeRotate_3_Source.bmp", imgSource);
      return Image;
  }

  // Rotation of scalled image
  cv::Mat DeRotate_5(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
  {
    cv::Point2f dw = ROIRect[3] - ROIRect[0];
    float width = sqrt(dw.x*dw.x + dw.y*dw.y);
    cv::Point2f dh = ROIRect[1] - ROIRect[0];
    float height = sqrt(dh.x*dh.x + dh.y*dh.y);

    int n = static_cast<int>(round(height)), m = static_cast<int>(round(width));

    cv::Mat ScalledImage;
    int interpolation = cv::INTER_LINEAR;
    if (Scale > 0 && Scale != 1.0f) resize(imgSource, ScalledImage, cv::Size(0, 0), Scale, Scale, interpolation);

    cv::Mat Result(n + 1, m + 1, CV_8UC3, cv::Scalar(0, 0, 0));

    dw = dw*Scale*(1.0f / width);
    dh = dh*Scale*(1.0f / height);

    cv::Point2f p0(ROIRect[0].x*Scale, ROIRect[0].y*Scale);
    cv::Point2f currentPoint(p0);
      
    for (int i = 0; i < n; i++)
      for (int k = 0; k < m; k++)
      {
        std::vector<cv::Vec3b> colors;
        std::vector<int> counts;

        Vec3f colorsSum(0.0f, 0.0f, 0.0f);
        int count = 0;
        for (float p = -0.2*Scale; p < 0.2*Scale; p++)
          for (float t = -0.2*Scale; t < 0.2*Scale; t++)
          {
            currentPoint = p0 + static_cast<float>(i)*dh + static_cast<float>(k)*dw + /*0.5f*(dh + dw) + */cv::Point2f(p, t);
            if (currentPoint.x >= 0 && currentPoint.x < ScalledImage.cols && currentPoint.y >= 0 && currentPoint.y < ScalledImage.rows)
            {
              colorsSum += ScalledImage.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x)));			
              count++;
            }
            Result.at<cv::Vec3b>(i, k) = colorsSum / count;
          }
       }

    imwrite("DeRotate_5_ScalledImage.bmp", ScalledImage);
    ScalledImage.release();

    return Result;
  }

  // Rotation of scalled image
  cv::Mat DeRotate_5_1(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
  {
    cv::Point2d dw = ROIRect[3] - ROIRect[0];
    double width = sqrt(dw.x*dw.x + dw.y*dw.y);
    cv::Point2d dh = ROIRect[1] - ROIRect[0];
    double height = sqrt(dh.x*dh.x + dh.y*dh.y);

    int n = static_cast<int>(round(height)), m = static_cast<int>(round(width));

    cv::Mat Result(n + 1, m + 1, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat ScalledImage;
    int interpolation = cv::INTER_LINEAR;
    if (Scale > 0 && Scale != 1.0f) resize(imgSource, ScalledImage, cv::Size(0, 0), Scale, Scale, interpolation);

    dw = dw*Scale*(1.0f / width);
    dh = dh*Scale*(1.0f / height);

    cv::Point2d p0(round(ROIRect[0].x*Scale), round(ROIRect[0].y*Scale));
    cv::Point2d currentPoint(p0);
      
    for (int i = 0; i < n; i++)
      for (int k = 0; k < m; k++)
      {
        std::vector<cv::Vec3b> colors;
        std::vector<int> counts;

        Vec3f colorsSum(0.0f, 0.0f, 0.0f);
        int count = 0;

        currentPoint = p0 + static_cast<float>(i)*dh + static_cast<float>(k)*dw;
        if (currentPoint.x >= 0 && currentPoint.x < ScalledImage.cols && currentPoint.y >= 0 && currentPoint.y < ScalledImage.rows)
          {
            colorsSum += ScalledImage.at<cv::Vec3b>(int(std::round(currentPoint.y)), int(std::round(currentPoint.x)));            
            count++;
          }

        Result.at<cv::Vec3b>(i, k) = colorsSum / count;
      }

    imwrite("DeRotate_5_1_ScalledImage.bmp", ScalledImage);

    ScalledImage.release();

    return Result;
  }

  cv::Mat DeRotate_withScale(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect1, float Scale)
  {
      Mat ImageCopy;
      Mat Result;

      int interpolation = cv::INTER_LINEAR;// INTER_CUBIC; //INTER_AREA; //INTER_NEAREST; //INTER_LINEAR;
      if (Scale > 0 && Scale != 1.0f) resize(imgSource, ImageCopy, cv::Size(0, 0), Scale, Scale, interpolation);

      vector<Point2f> ROIRect(4);
      for (unsigned int i = 0; i < ROIRect1.size(); i++)
        ROIRect[i] = Scale*ROIRect1[i];
      cv::Point2f dw = ROIRect[3] - ROIRect[0];
      float width = sqrt(dw.x*dw.x + dw.y*dw.y);
      cv::Point2f dh = ROIRect[1] - ROIRect[0];
      float height = sqrt(dh.x*dh.x + dh.y*dh.y);

      int n = static_cast<int>(round(height)), m = static_cast<int>(round(width));
      cv::Mat Image = cv::Mat(n + 1, m + 1, CV_8UC3, cv::Scalar(0, 0, 0));

      dw = dw*(1.0f / width);
      dh = dh*(1.0f / height);

      cv::Point2f currentPoint = ROIRect[0];
      for (int i = 0; i <= n; i++)
        for (int k = 0; k <= m; k++)
        {
          currentPoint = ROIRect[0] + static_cast<float>(i)*dh + static_cast<float>(k)*dw;
          Image.at<cv::Vec3b>(i, k) = ImageCopy.at<cv::Vec3b>(int(std::roundf(currentPoint.y)), int(std::roundf(currentPoint.x)));
        }

      imwrite("DeRotate_withScale_DerotatedImage.bmp", Image);
      interpolation = cv::INTER_NEAREST;
      if (Scale > 0 && Scale != 1.0f) resize(Image, Result, cv::Size(0, 0), 1.0f / Scale, 1.0f / Scale, interpolation);

      return Result;
  }

  // Copying the image palette
  std::vector<cv::Vec3b> CopyColors(const cv::Mat &imgSource)
  {

    cv::Size size = imgSource.size();
    int N = size.height;
    int M = size.width;
    std::vector<cv::Vec3b> colors;
    for (int i = 0; i < N; i++)
      for (int k = 0; k < M; k++)
      {
        int Q = -1;
        cv::Vec3b color = imgSource.at<cv::Vec3b>(i, k);
        if(colors.size() > 0)
        for (int q = 0; q < colors.size(); q++)
          if (colors[q] == color) Q = q;

        if (Q == -1)
          colors.push_back(color);
       }
    colors.push_back(Vec3b(0, 0, 0));
    return colors;
  }

  // Restore the image palette
  void RestoreColors(cv::Mat &image, std::vector<cv::Vec3b> colors)
  {
    cv::Size size = image.size();
    int N = size.height;
    int M = size.width;

    for (int i = 0; i < N; i++)
      for (int k = 0; k < M; k++)
      {
        int Q = -1;
        double min = 3 * 255 * 255;
        cv::Vec3b color = image.at<cv::Vec3b>(i, k);
        for (int q = 0; q < colors.size(); q++)
        {
          cv::Vec3b colorDist = color - colors[q];
          double D = 0;
          for (int t = 0; t < 3; t++)
            D = D + static_cast<float>(colorDist[t] * colorDist[t]);
          if (D < min)
          {
            Q = q;
            min = D;
          }
        }
        image.at<cv::Vec3b>(i, k) = colors[Q];
      }
  }


 
 cv::Mat DeRotate_8(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
  {    
    std::vector<cv::Vec3b> colors = CopyColors(imgSource); // Save palette
      
    cv::Mat Result = DeRotate_7_1(imgSource, ROIRect, 11.0f); // Image rotation
    //DeRotate_withScale(imgSource, ROIRect, 11.0f);//Result = DeRotate_5(imgSource, ROIRect, 11.0f);	  
    RestoreColors(Result, colors); // Restore palette

    return Result;
  }

 cv::Mat DeRotate_8_1(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
 {
     std::vector<cv::Vec3b> colors = CopyColors(imgSource); // Save palette
     cv::Mat Result = DeRotate_withScale(imgSource, ROIRect, 11.0f); // Image rotation															     
     RestoreColors(Result, colors); // Restore palette

     return Result;
 }

 
 cv::Mat DeRotate_8_2(const cv::Mat &imgSource, std::vector<cv::Point2f> &ROIRect, float Scale)
 {
     std::vector<cv::Vec3b> colors = CopyColors(imgSource); // Save palette
     cv::Mat Result = DeRotate_5(imgSource, ROIRect, 11.0f); // Image rotation															   
     RestoreColors(Result, colors); // Restore palette

     return Result;
 }
  POSERECT<Point2f> CreateRect1(float x1, float x2, float y1, float y2)
  {
      Point2f a(x2, y1), b(x2, y2), c(x1, y2), d(x1, y1);
      POSERECT <Point2f> rect(a, b, c, d);
      return rect;
  }

  // Rotation of the rectangle around center 
  POSERECT<Point2f> RotateRect1(POSERECT<Point2f> &rect, float angle)
  {
      POSERECT<Point2f> RotatedRect;
      Point2f center = rect.GetCenter<Point2f>();
      RotatedRect.point1 = spelHelper::rotatePoint2D(rect.point1, center, angle);
      RotatedRect.point2 = spelHelper::rotatePoint2D(rect.point2, center, angle);
      RotatedRect.point3 = spelHelper::rotatePoint2D(rect.point3, center, angle);
      RotatedRect.point4 = spelHelper::rotatePoint2D(rect.point4, center, angle);
      return RotatedRect;
  }

//===============================================================================================================================
// TESTING

  cv::Rect SelectMaskRect(const cv::Mat &Image, uchar Q)
  {
    cv::Size size = Image.size();
    Point2i p0(size.width, size.height), p1(0, 0);
    
    for (int x = 0; x < size.width; x++)
      for (int y = 0; y < size.height; y++)
      {
        cv::Vec3b color = Image.at<cv::Vec3b>(y, x);

        if ((color.val[0] > Q) || (color.val[1] > Q) || (color.val[2] > Q))
        {
          if (x < p0.x) p0.x = x;
          if (y < p0.y) p0.y = y;
          if (x > p1.x) p1.x = x;
          if (y > p1.y) p1.y = y;
        }
      }
    cv::Rect rect(p0, p1 + cv::Point2i(1,1));

    return rect;
  }

  pair<float, Point2i> CompareImages(Mat pattern, Mat image, bool useColorDistScore = false, bool considerExcess =true, int Q = 10 )
  {
    cv::Size pSize = pattern.size(), iSize = image.size(); 
    Point2i d(iSize.width - pSize.width, iSize.height - pSize.height);

    Point2i shift(0, 0);
    double maxScore = 0;
    for (int i = 0; i <= d.y; i++)
      for (int k = 0; k <= d.x; k++)
      {
        double score = 0, ColorScore = 0;
        int patternPoints = 0;
        for(int x = 0; x < iSize.width; x++)
          for(int y = 0; y < iSize.height; y++)
          {
            Point2i p(x - k, y - i);
            Vec3b iColor = image.at<Vec3b>(y, x);
            if( (p.x >= 0) && (p.y >= 0) && (p.x < pSize.width) && (p.y < pSize.height))
            {
              Vec3b pColor = pattern.at<Vec3b>(p.y, p.x);
              patternPoints++;
              Vec3b D = pColor - iColor;
              float colorDist = sqrt(D.val[0] * D.val[0] + D.val[1] * D.val[1] + D.val[2] * D.val[2]);
              ColorScore = ColorScore + (colorDist / (255.0*sqrt(3.0)));
              if (colorDist < Q) score++;
            }
            else
            {
              if(considerExcess)
                if(iColor.val[0] >=Q || iColor.val[1] >= Q || iColor.val[2] >= Q)
                {
                  score--;
                  ColorScore = ColorScore + sqrt(iColor.val[0]* iColor.val[0] + iColor.val[1]* iColor.val[1] + iColor.val[2]* iColor.val[2]/3.0)/255.0;
                }
            }
          }
        score = score / static_cast<double>(pSize.width*pSize.height);
        if (useColorDistScore)
          score = 1.0 - ColorScore / static_cast<double>(pSize.width*pSize.height);
        if (score > maxScore)
        {
          maxScore = score;
          shift = Point2i(k, i);
        }
      }
    return pair<float, Point2i>(maxScore, shift);
  }

  Size MaxSize(vector<Mat> images, Size maxSize = Size(0,0))
  {
    vector<Rect> ImageROI;
    Size max_size = maxSize;
    for (int i = 0; i < images.size(); i++)
    {
      ImageROI.push_back(SelectMaskRect(images[i], 10));
      if(ImageROI[i].width > max_size.width)
        max_size.width = ImageROI[i].width;
      if (ImageROI[i].height > max_size.height)
        max_size.height = ImageROI[i].height;
    }
    return max_size + Size(2, 2);
  }


  Size maxTextSize(vector<String> names, int fontFace, double fontScale, int * baseLine, int thickness = 1)
  {
    Size max_size(0, 0);
    for( int i = 0; i < names.size(); i++)
    {
      Size textSize = getTextSize(names[i] + "__", fontFace, fontScale, thickness, baseLine);
      if(textSize.width > max_size.width)
        max_size.width = textSize.width;
      if (textSize.height > max_size.height)
        max_size.height = textSize.height;
    }
    return max_size;
  }

  Mat X0(int columns, Size CellSize, Size borderSize,Size textSize, /* vector<Mat> images, String name,*/ vector<String> names, float scale = 2.0f)
  {
    Size max_size(scale * CellSize.height + borderSize.height, scale * CellSize.width + borderSize.width);

    Mat X = Mat((names.size() + 2)*max_size.height, textSize.width + columns*max_size.width, CV_8UC3, Scalar(0, 0, 0));

    for(int i = 0; i < columns; i++)
      for(int y = 0; y < X.size().height; y++)
        X.at<Vec3b>(y, textSize.width + i*(max_size.width)) = Vec3b(255, 0, 0);

    for (int i = 0; i < names.size() + 2; i++)
      for (int x = 0; x < X.size().width; x++)
        X.at<Vec3b>(i*(max_size.height), x) = Vec3b(255, 0, 0);
    stringstream s;
    s << scale;
    cv::putText(X, " Patterns", Point(0, 17 + 0 * max_size.height), 5, 0.65f, CV_RGB(0, 193, 0));
    cv::putText(X, " scale = " + s.str(), Point(0, 10 + 3*textSize.height + 0 * max_size.height), 5, 0.65f, CV_RGB(255, 0, 0));
    cv::putText(X, " Samples", Point(0, 17 + 1*max_size.height), 5, 0.65f, CV_RGB(0, 193, 0));
    s.clear();
    for (int i = 0; i < names.size() ; i++)
      cv::putText(X, " " + names[i], Point(0, 17 + (i + 2)*max_size.height), 5, 0.65f, CV_RGB(0, 193, 0));

    return X;
  }

  void PutImages(Mat &Result, int row, vector<Mat> images, Size max_size, Size borderSize, Size textSize, float scale)
  {
     Size CellSize(scale * max_size.height + borderSize.height, scale * max_size.width + borderSize.width);
     for (unsigned int i = 0; i < images.size(); i++)
     {
       Mat tempImage = images[i](SelectMaskRect(images[i], 10));
       resize(tempImage, tempImage, cv::Size(0, 0), scale, scale, cv::INTER_NEAREST);
       for (int y = 0; y < tempImage.size().height; y++)
         for (int x = 0; x < tempImage.size().width; x++)
           Result.at<Vec3b>(row *CellSize.height + 0.5*(borderSize.height + CellSize.height - tempImage.size().height) + y, textSize.width + i*CellSize.width + 0.5*(borderSize.width + CellSize.width - tempImage.size().width) + x) = tempImage.at<Vec3b>(y, x);
      }
  }

  TEST(ImageRotationExperiments, SelectMaskRect)
  {
      Mat image(100, 100, CV_8UC3, Scalar(0, 0, 0));
      image.at<Vec3b>(10, 10) = Vec3b(255, 255, 255);
      image.at<Vec3b>(20, 20) = Vec3b(255, 255, 255);
      Rect rect = SelectMaskRect(image, 2);

      EXPECT_EQ(Rect(10, 10, 11, 11), rect);
      imwrite("temp150616.bmp", image(rect));
      image.release();
  }

  TEST(ImageRotationExperiments, ResizeSlandedROI)
  {
    float acceptableError = 0.1f;

    // Aclinic polygon
    vector<Point2f> InitialPolygon = { Point2f(0.0f, 0.5f), Point2f(2.0f, 0.5f), Point2f(2.0f, -0.5f), Point2f(0.0f, -0.5f) };	
    vector<Point2f> ExpectedPolygon = { Point2f(-0.5f, 1.0f), Point2f(2.5f, 1.0f), Point2f(2.5f, -1.0f), Point2f(-0.5f, -1.0f) };
    vector<Point2f> ActualPolygon = ResizeSlantedROI(InitialPolygon, cv::Size(2.0f, 3.0f));
    EXPECT_EQ(ExpectedPolygon, ActualPolygon);

    // Slanded polygon (45 degrees)
    InitialPolygon = { Point2f(7.0f, 3.0f), Point2f(12.0f, 8.0f), Point2f(9.0f, 11.0f), Point2f(4.0f, 6.0f) };
    ExpectedPolygon = { Point2f(7.3f, 1.3f), Point2f(13.7f, 7.7f), Point2f(8.7f, 12.7f), Point2f(2.3f, 6.3f) };
    ActualPolygon = ResizeSlantedROI(InitialPolygon, cv::Size(7, 9));
    for (unsigned int i = 0; i < ExpectedPolygon.size(); i++)
    {
      EXPECT_NEAR(ExpectedPolygon[i].x, ActualPolygon[i].x, acceptableError) << "45 degrees slanded polygon, point " << i << endl;
      EXPECT_NEAR(ExpectedPolygon[i].y, ActualPolygon[i].y, acceptableError) << "45 degrees slanded polygon, point " << i << endl;
    }

    // Slanded polygon (314 degrees)
    InitialPolygon = { Point2f(9.0f, 11.0f), Point2f(4.0f, 6.0f), Point2f(7.0f, 3.0f), Point2f(12.0f, 8.0f) };
    ExpectedPolygon = { Point2f(8.7f, 12.7f), Point2f(2.3f, 6.3f), Point2f(7.3f, 1.3f), Point2f(13.7f, 7.7f) };
    ActualPolygon = ResizeSlantedROI(InitialPolygon, cv::Size(7, 9));
    for (unsigned int i = 0; i < ExpectedPolygon.size(); i++)
    {
      EXPECT_NEAR(ExpectedPolygon[i].x, ActualPolygon[i].x, acceptableError) << "314 degrees slanded polygon, point " << i << endl;
      EXPECT_NEAR(ExpectedPolygon[i].y, ActualPolygon[i].y, acceptableError) << "314 degrees slanded polygon, point " << i << endl;
    }
  }


  TEST(ImageRotationExperiments, ExtendSlandedROI)
  {
    //float acceptableError = 0.1f;

    // Aclinic polygon
    vector<Point2f> InitialPolygon = { Point2f(0.0f, 0.5f), Point2f(2.0f, 0.5f), Point2f(2.0f, -0.5f), Point2f(0.0f, -0.5f) };
    vector<Point2f> ExpectedPolygon = { Point2f(-0.5f, 1.0f), Point2f(2.5f, 1.0f), Point2f(2.5f, -1.0f), Point2f(-0.5f, -1.0f) };
    vector<Point2f> ActualPolygon = ExtendSlantedROI(InitialPolygon, 0.5f, 0.5f);
    EXPECT_EQ(ExpectedPolygon, ActualPolygon) << "- aclinic polygon" << endl;

    // Slanded polygon (45 degrees)
    InitialPolygon = { Point2f(7.0f, 3.0f), Point2f(12.0f, 8.0f), Point2f(9.0f, 11.0f), Point2f(4.0f, 6.0f) };
    ExpectedPolygon = { Point2f(7.0f, 1.0f), Point2f(14.0f, 8.0f), Point2f(9.0f, 13.0f), Point2f(2.0f, 6.0f) };
    ActualPolygon = ExtendSlantedROI(InitialPolygon, sqrt(2.0), sqrt(2.0));
    EXPECT_EQ(ExpectedPolygon, ActualPolygon);
    /*for (unsigned int i = 0; i < ExpectedPolygon.size(); i++)
    {
      EXPECT_NEAR(ExpectedPolygon[i].x, ActualPolygon[i].x, acceptableError) << "45 degrees slanded polygon, point " << i << endl;
      EXPECT_NEAR(ExpectedPolygon[i].y, ActualPolygon[i].y, acceptableError) << "45 degrees slanded polygon, point " << i << endl;
    }*/

    // Slanded polygon (314 degrees)
    InitialPolygon = { Point2f(9.0f, 11.0f), Point2f(4.0f, 6.0f), Point2f(7.0f, 3.0f), Point2f(12.0f, 8.0f) };
    ExpectedPolygon = { Point2f(9.0f, 13.0f), Point2f(2.0f, 6.0f), Point2f(7.0f, 1.0f), Point2f(14.0f, 8.0f) };
    ActualPolygon = ExtendSlantedROI(InitialPolygon, sqrt(2.0), sqrt(2.0));
    EXPECT_EQ(ExpectedPolygon, ActualPolygon);
    /*for (unsigned int i = 0; i < ExpectedPolygon.size(); i++)
    {
      EXPECT_NEAR(ExpectedPolygon[i].x, ActualPolygon[i].x, acceptableError) << "314 degrees slanded polygon, point " << i << endl;
      EXPECT_NEAR(ExpectedPolygon[i].y, ActualPolygon[i].y, acceptableError) << "314 degrees slanded polygon, point " << i << endl;
    }*/
  }

  TEST(ImageRotationExperiments, DeRotate_All)
  {
    // Prepare test data
    String Path;
#ifdef WINDOWS
#ifdef DEBUG
    if (IsDebuggerPresent())
      Path = "Debug/speltests_TestData/ImageRotationTestData/";
    else
      Path = "speltests_TestData/ImageRotationTestData/";
#else
    Path = "Release/speltests_TestData/ImageRotationTestData/";
#endif  // DEBUG
#else
    Path = "speltests_TestData/ImageRotationTestData/";
#endif 

    // Read images
    vector<Mat> images;
    vector<Mat> patterns;
    vector<Mat> results;
    vector<Mat> selectedImages;
    vector<Mat> selectedResults;
    vector<long int> time;
    vector<cv::Size> sizes;
    vector<POSERECT <Point2f>> Rects;

    int samplesCount = 4;

    for (int i = 1; i <= samplesCount; i++)
    {
      stringstream temp;
      temp << i;
      images.push_back(imread(Path + "Q" + temp.str() + ".jpg"));
      patterns.push_back(imread(Path + "Pattern" + temp.str() + ".bmp"));
      temp.clear();
    }
        
    // Create ROI polygon
    float angle = 45; // the rotation angle
    float x1 = 9.0, x2 = 39.0, y1 = 19.0, y2 = 59.0; // the rectangle vertices
    POSERECT <Point2f> rect = CreateRect1(x1, x2, y1, y2);
    POSERECT <Point2f> RotatedRect = RotateRect1(rect, angle);
    for (int i = 0; i < images.size(); i++)
      Rects.push_back(RotatedRect);

    // Functions
    vector<Mat(*)(const cv::Mat &, std::vector<cv::Point2f> &, float)> functions = {RotateImageToDefault, 
        DeRotate_0, DeRotate_7, DeRotate_7_1, DeRotate_2_visualization, DeRotate_3_visualization, DeRotate_4_visualization,
        DeRotate_5, DeRotate_5_1, DeRotate_withScale, DeRotate_8, DeRotate_8_1, DeRotate_8_2 };
    vector<String> names = { "RotateImageToDefault", 
        "DeRotate_0", "DeRotate_7", "DeRotate_7_1", "DeRotate_2_visualisation", "DeRotate_3_visualisation", "DeRotate_4_visualisation",
         "DeRotate_5", "DeRotate_5_1", "DeRotate_withScale","DeRotate_8", "DeRotate_8_1" ,"DeRotate_8_2" };

    // Prepare visualization
    Size max_size = MaxSize(images);
    max_size = MaxSize(images, max_size);
 
    Size borderSize(0, 0);

    int fontFace = 5;
    double fontScale = 0.65;
    int *text_y = new int;

    Size textSize = maxTextSize(names, fontFace, fontScale, text_y);
    float scale = 2.0f;
    int columns = images.size();
    Mat X = X0(columns, max_size, borderSize, textSize, names, scale);
    
    PutImages(X, 0, patterns, max_size, borderSize, textSize, scale);
    PutImages(X, 1, images, max_size, borderSize, textSize, scale);

    
    // Call all rotation functions
    for (unsigned int f = 0; f < functions.size(); f++)
    {
      long t0, t1;
      for (unsigned int i = 0; i < images.size(); i++)
      {
        stringstream temp;
        temp << names[f] + "(image" << i << ")";

        Mat deRotatedImage;
        vector<Point2f> StandledROI = ExtendSlantedROI(Rects[i].asVector(), 2, 2);
        t0 = clock();// GetTickCount();
        deRotatedImage = functions[f](images[i], RotatedRect.asVector(), 11.0f);
        //deRotatedImage = functions[f](images[i], StandledROI, 11.0f);
        t1 = clock();// GetTickCount();

        sizes.push_back(images[i].size());
        //imwrite(temp.str() + ".bmp", deRotatedImage);
        results.push_back(deRotatedImage.clone());
        deRotatedImage.release();
        
        // Visualization
        cv::Rect MaskRect = SelectMaskRect(results[i], 10);
        //cout << MaskRect << endl;
        selectedResults.push_back(results[i](MaskRect).clone());
        //imwrite(temp.str()+"SelectedROI.bmp", selectedResults[i]);

        // Put results
        pair<float, Point2i> score, colorScore;
        score = CompareImages(patterns[i], results[i], 0);
        colorScore = CompareImages(patterns[i], results[i], 1);
        cout << temp.str() << ": score = " << score.first /*<< ", shift = " << score.second*/;
        cout << ", colorScore = " << colorScore.first << ", ResultSize = "  << selectedResults[i].size() << /*", shift =" << colorScore.second <<*/ ", time = " << t1 - t0 << endl;
        temp.clear();
      }
      cout << endl;

      // Visualization
      PutImages(X, f + 2, selectedResults, max_size, borderSize, textSize, scale);

      // Clear
      for (unsigned int i = 0; i < images.size(); i++)
      {
        results[i].release();
        selectedResults[i].release();
      }
      results.clear();
      selectedResults.clear();
    }

    cout << "Results in 'ImageRotationExperiments.bmp'" << endl << endl;
    imwrite("ImageRotationExperiments.bmp", X);
    waitKey(0);
    X.release();

    
    // Clear
    for (unsigned int i = 0; i < images.size(); i++)
    {
      images[i].release();
      patterns[i].release();
      results[i].release();
      selectedResults[i].release();
    }
    images.clear();
    patterns.clear();
    results.clear();
    Rects.clear();
    selectedResults.clear();
  }

TEST(ImageRotationExperiments, DeRotate_All_extendedROI)
  {
    // Prepare test data
    String Path;
#ifdef WINDOWS
#ifdef DEBUG
    if (IsDebuggerPresent())
      Path = "Debug/speltests_TestData/ImageRotationTestData/";
    else
      Path = "speltests_TestData/ImageRotationTestData/";
#else
    Path = "Release/speltests_TestData/ImageRotationTestData/";
#endif  // DEBUG
#else
    Path = "speltests_TestData/ImageRotationTestData/";
#endif 

    // Read images
    vector<Mat> images;
    vector<Mat> patterns;
    vector<Mat> results;
    vector<Mat> selectedImages;
    vector<Mat> selectedResults;
    vector<long int> time;
    vector<cv::Size> sizes;
    vector<POSERECT <Point2f>> Rects;

    int samplesCount = 4;

    for (int i = 1; i <= samplesCount; i++)
    {
      stringstream temp;
      temp << i;
      images.push_back(imread(Path + "Q" + temp.str() + ".jpg"));
      patterns.push_back(imread(Path + "Pattern" + temp.str() + ".bmp"));
      temp.clear();
    }
    
    
    // Create ROI polygon
    float angle = 45; // the rotation angle
    float x1 = 9.0, x2 = 39.0, y1 = 19.0, y2 = 59.0; // the rectangle vertices
    POSERECT <Point2f> rect = CreateRect1(x1, x2, y1, y2);
    POSERECT <Point2f> RotatedRect = RotateRect1(rect, angle);
    for (int i = 0; i < images.size(); i++)
      Rects.push_back(RotatedRect);

    // Functions
    vector<Mat(*)(const cv::Mat &, std::vector<cv::Point2f> &, float)> functions = {RotateImageToDefault, 
        DeRotate_0, DeRotate_7, DeRotate_7_1, DeRotate_2_visualization, DeRotate_3_visualization, DeRotate_4_visualization,
        DeRotate_5, DeRotate_5_1, DeRotate_withScale, DeRotate_8, DeRotate_8_1, DeRotate_8_2 };
    vector<String> names = { "RotateImageToDefault", 
        "DeRotate_0", "DeRotate_7", "DeRotate_7_1", "DeRotate_2_visualisation", "DeRotate_3_visualisation", "DeRotate_4_visualisation",
         "DeRotate_5", "DeRotate_5_1", "DeRotate_withScale","DeRotate_8", "DeRotate_8_1" ,"DeRotate_8_2" };

    // Prepare visualization
    Size max_size = MaxSize(images);
    max_size = MaxSize(images, max_size);
 
    Size borderSize(0, 0);

    int fontFace = 5;
    double fontScale = 0.65;
    int *text_y = new int;

    Size textSize = maxTextSize(names, fontFace, fontScale, text_y);
    float scale = 2.0f;
    int columns = images.size();
    Mat X = X0(columns, max_size, borderSize, textSize, names, scale);
    
    PutImages(X, 0, patterns, max_size, borderSize, textSize, scale);
    PutImages(X, 1, images, max_size, borderSize, textSize, scale);
    
    // Call all rotation functions
    for (unsigned int f = 0; f < functions.size(); f++)
    {
      long int t0, t1;
      for (unsigned int i = 0; i < images.size(); i++)
      {
        Mat deRotatedImage;
        vector<Point2f> StandledROI = ExtendSlantedROI(Rects[i].asVector(), 2, 2);
        t0 = clock();
        deRotatedImage = functions[f](images[i], StandledROI, 11.0f);
        t1 = clock();

        sizes.push_back(images[i].size());
        results.push_back(deRotatedImage.clone());
        deRotatedImage.release();
        
        // Visualization
        cv::Rect MaskRect = SelectMaskRect(results[i], 10);
        selectedResults.push_back(results[i](MaskRect).clone());

        // Put results
        pair<float, Point2i> score, colorScore;
        score = CompareImages(patterns[i], results[i], 0);
        colorScore = CompareImages(patterns[i], results[i], 1);
        cout << names[f] + "(image" << i << ")" << ": score = " << score.first << ", shift = " << score.second;
        cout << ";   colorScore = " << colorScore.first << ", shift =" << colorScore.second << ";   ResultSize = " << selectedResults[i].size() << ";   time = " << t1 - t0 << endl;
      }
      cout << endl;

      // Visualization
      PutImages(X, f + 2, selectedResults, max_size, borderSize, textSize, scale);

      // Clear
      for (unsigned int i = 0; i < images.size(); i++)
      {
        results[i].release();
        selectedResults[i].release();
      }
      results.clear();
      selectedResults.clear();
    }

    cout << "Results in 'ImageRotationExperiments-ExtendedROI.bmp'" << endl << endl;
    imwrite("ImageRotationExperiments-ExtendedROI.bmp", X);
    waitKey(0);
    X.release();

    // Clear
    for (unsigned int i = 0; i < images.size(); i++)
    {
      images[i].release();
      patterns[i].release();
      results[i].release();
      selectedResults[i].release();
    }
    images.clear();
    patterns.clear();
    results.clear();
    Rects.clear();
    selectedResults.clear();
  }

}