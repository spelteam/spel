#include "spelGeometry.hpp"

namespace SPEL
{
//ROI

  // Matrixes for search contour operations
  const uchar C_[3][3] = { { 7, 0, 1 },{ 6, 0, 2 },{ 5, 4, 3 } }; 
  const cv::Point2i P_[8] = { cv::Point2i(0, -1), cv::Point2i(1, -1), cv::Point2i(1, 0), cv::Point2i(1, 1), cv::Point2i(0, 1), cv::Point2i(-1, 1), cv::Point2i(-1, 0), cv::Point2i(-1, -1) };

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
  std::vector<cv::Point2i> SearchROI(cv::Mat mask)
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

    //cv::Rect maskROI(temp[0], temp[1]);
    /*cv::Size borderSize = cv::Size(5, 5);
    borderSize += borderSize;
    maskROI = resizeROI_(maskROI, maskROI.size() + borderSize, mask.size());*/

    return temp; // vector of ROI endpoints
  }

  cv::Rect toROIRect(std::vector<cv::Point2i> endpoints)
  {
    cv::Rect maskROI(endpoints[0], endpoints[1]);
    return maskROI;
  }

  // Search center of mask in the ROI
  // "ROIEndpoints" - endpoints of mask ROI: {topLeft, bottomRight}
  // "colorThreshold" -  max value of background color, white pixel threshold
  cv::Point2i MaskCenter(cv::Mat mask, std::vector<cv::Point2i> ROIEndpoints, uchar colorThreshold)
  {
    double N = 0;
    cv::Point2d center(0.0, 0.0);
    for (int y = ROIEndpoints[0].y; y <= ROIEndpoints[1].y; y++ )
      for (int x = ROIEndpoints[0].x; x <= ROIEndpoints[1].x; x++)
      {
        if(mask.at<uchar>(y,x) > colorThreshold)
        { 
          N++;
          center = center + cv::Point2d(x, y);
        }         
      }

    return cv::Point2i(static_cast<int>(round(center.x/ N)), static_cast<int>(round(center.y/N)));
  }

  // Search center of mask in the ROI
  // "colorThreshold" -  max value of background color, white pixel threshold
  cv::Point2i MaskCenter(cv::Mat mask, cv::Rect ROIRect, uchar colorThreshold)
  {
    cv::Point2i p0(ROIRect.x, ROIRect.y);
    cv::Point2i p1(ROIRect.x + ROIRect.width, ROIRect.y + ROIRect.height);
    std::vector<cv::Point2i> endpoints = { p1, p0 };

    return MaskCenter(mask, endpoints, colorThreshold);
  }

  // ?
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

//PartPolygons

  std::vector<cv::Point2f> buildPartPolygon(float LWRatio, cv::Point2f p0, cv::Point2f p1)
  {
    std::vector<cv::Point2f> partRect;
    cv::Point2f d = p0 - p1;
    //if (LWRatio <=0) DebugMessage(" LWRatio <= 0", 2);
    if (LWRatio > 0)
    {
      float k = 0.5f/LWRatio;
      float dx = k*(d.y);
      float dy = k*(-d.x);
      partRect.push_back(cv::Point2f(p0.x + dx, p0.y + dy));
      partRect.push_back(cv::Point2f(p1.x + dx, p1.y + dy));
      partRect.push_back(cv::Point2f(p1.x - dx, p1.y - dy));
      partRect.push_back(cv::Point2f(p0.x - dx, p0.y - dy));
    }

    return partRect;
  }

  float getLenght(std::vector<cv::Point2f> partPolygon)
  {
    cv::Point2f d = partPolygon[1] - partPolygon[0];
    return sqrt(d.x*d.x + d.y*d.y);
  }

  float getWidth(std::vector<cv::Point2f> partPolygon)
  {
    cv::Point2f d = partPolygon[3] - partPolygon[0];
    return sqrt(d.x*d.x + d.y*d.y);
  }

  std::map<int, std::vector<cv::Point2f>> getAllPolygons(Skeleton &skeleton)
  {
    std::map<int, std::vector<cv::Point2f>> Rects;
    tree<BodyPart> PartTree = skeleton.getPartTree();
    for (tree<BodyPart>::iterator BP_iterator = PartTree.begin(); BP_iterator != PartTree.end(); BP_iterator++)
    {
      BodyJoint *j0 = skeleton.getBodyJoint(BP_iterator->getParentJoint());
      BodyJoint *j1 = skeleton.getBodyJoint(BP_iterator->getChildJoint());
      std::vector<cv::Point2f> Rect = buildPartPolygon(BP_iterator->getLWRatio(), j0->getImageLocation(), j1->getImageLocation());
      Rects.emplace(std::pair<int, std::vector<cv::Point2f>>((*BP_iterator).getPartID(), Rect));
    }
    return Rects;
  }

//Skeleton

  Skeleton operator+(Skeleton s1, Skeleton s2)
  {
    Skeleton s;
    tree<BodyJoint> joints1 = s1.getJointTree(); 
    tree<BodyJoint> joints2 = s2.getJointTree();
    if (joints1.size() == joints2.size())
    {
      tree<BodyJoint>::iterator i1 = joints1.begin();
      tree<BodyJoint>::iterator i2 = joints2.begin();
      for (i1; i1 != joints1.end(); i1++)
      {
        cv::Point2f a = i1->getImageLocation() + i2->getImageLocation();
        cv::Point3f b = i1->getSpaceLocation() + i2->getSpaceLocation();
        i1->setImageLocation(a);
        i1->setSpaceLocation(b);
        i2++;
      }
      s = s1;
      s.setJointTree(joints1);
    }
    
    return s;
  }

  Skeleton operator+(Skeleton s, cv::Point2f P)
  {
    Skeleton s1;
    tree<BodyJoint> joints1 = s.getJointTree(); 
    tree<BodyJoint>::iterator i1 = joints1.begin();
    for (i1; i1 != joints1.end(); i1++)
    {
      cv::Point2f a = i1->getImageLocation() + P;
      cv::Point3f b = i1->getSpaceLocation() + cv::Point3f(P.x, P.y, 0.0f);
      i1->setImageLocation(a);
      i1->setSpaceLocation(b);
    }
    s1 = s;
    s1.setJointTree(joints1);

    return s1;
  }

  Skeleton operator+(cv::Point2f P, Skeleton s)
  {
    return (s + P);
  }

  Skeleton operator-(Skeleton s1, cv::Point2f P)
  {
    return s1+(-P);
  }

  Skeleton operator*(Skeleton s, float k)
  {
    Skeleton s1;
    tree<BodyJoint> joints1 = s.getJointTree();

    tree<BodyJoint>::iterator i1 = joints1.begin();
    for (i1; i1 != joints1.end(); i1++)
    {
      cv::Point2f a = i1->getImageLocation() * k;
      cv::Point3f b = i1->getSpaceLocation() * k;
      i1->setImageLocation(a);
      i1->setSpaceLocation(b);
    }
    s1 = s;
    s1.setJointTree(joints1);

    return s1;
  }

  Skeleton operator*(float k, Skeleton s)
  {
    return (s*k);
  }

  Skeleton operator/(Skeleton s, float k)
  {
    Skeleton s1;
    if(k != 0.0f) s1 = s*(1/k);

    return s1;
  }

//Visualization

  void PutPartRect(cv::Mat &Image, std::vector<cv::Point2f> polygon, cv::Scalar color)
  {
    polygon.push_back(polygon[0]);
    for (unsigned int i = 1; i < polygon.size(); i++)
      line(Image, polygon[i - 1], polygon[i], color, 1, 1);
  }

  void putSkeleton(cv::Mat image, Skeleton skeleton, cv::Scalar color)
  {
    std::map<int, std::vector<cv::Point2f>> polygons = getAllPolygons(skeleton);
    for (auto p = 0; p < polygons.size(); p++)
      PutPartRect(image, polygons[p], color);
  }

}