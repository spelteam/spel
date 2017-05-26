// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

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

  cv::Rect toROIRect(std::vector<cv::Point2f> endpoints)
  {
    cv::Point2i p0(static_cast<int>(trunc(endpoints[0].x)), static_cast<int>(trunc(endpoints[0].y)));
    cv::Point2i p1(static_cast<int>(ceil(endpoints[1].x)), static_cast<int>(ceil(endpoints[1].y)));
    cv::Rect maskROI(p0, p1);
    return maskROI;
  }

  // Search center of mask in the ROI
  // "ROIEndpoints" - endpoints of mask ROI: {topLeft, bottomRight}
  // "colorThreshold" -  max value of background color, white pixel threshold
  cv::Point2f MaskCenter(cv::Mat mask, std::vector<cv::Point2i> ROIEndpoints, uchar colorThreshold)
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
    if (N > 0)
      center = cv::Point2d(center.x / N, center.y / N);
    else
    {
      center = 0.5 * (ROIEndpoints[0] +  ROIEndpoints[1]);
      DebugMessage("Mask ROI is empty", 2);
    }

    return cv::Point2f(static_cast<float>(center.x), static_cast<float>(center.y));
  }

  // Search center of mask in the ROI
  // "colorThreshold" -  max value of background color, white pixel threshold
  cv::Point2f MaskCenter(cv::Mat mask, cv::Rect ROIRect, uchar colorThreshold)
  {
    cv::Point2i p0(ROIRect.x, ROIRect.y);
    cv::Point2i p1(ROIRect.x + ROIRect.width - 1, ROIRect.y + ROIRect.height - 1);
    std::vector<cv::Point2i> endpoints = { p0, p1 };

    return MaskCenter(mask, endpoints, colorThreshold);
  }

  // == cv::adjustROI ?
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

  bool correctROI(cv::Rect &ROI, cv::Size imageSize)
  {
    cv::Rect temp = ROI;
    if (temp.x < 0) temp.x = 0;
    if (temp.y < 0) temp.y = 0;
    if (temp.x >= imageSize.width) temp.x = imageSize.width - 1;
    if (temp.y >= imageSize.height) temp.y = imageSize.height - 1;
    if (temp.width < 0) temp.width = 0;
    if (temp.height < 0) temp.height = 0;
    if (temp.x + temp.width > imageSize.width) 
      temp.width = imageSize.width - temp.x;
    if (temp.y + temp.height > imageSize.height) 
      temp.height = imageSize.height - temp.y;

    bool b = (temp != ROI);
    if(b)
      DebugMessage("ROI out of range - fixed", 5);
  
    ROI = temp;
    return b;
  }

  bool correctEndpoints(std::vector<cv::Point2f> &endpoints, cv::Size imageSize)
  {
    std::vector<cv::Point2f> temp = endpoints;
    float f;
    if(temp[0].x > temp[1].x) 
    {
      f = temp[0].x;
      temp[0].x = temp[1].x;
      temp[1].x = f;
    }
    if (temp[0].y > temp[1].y)
    {
      f = temp[0].y;
      temp[0].y = temp[1].y;
      temp[1].y = f;
    }
    if (temp[0].x < 0) temp[0].x = 0;
    if (temp[0].y < 0) temp[0].y = 0;
    if (temp[1].x >= imageSize.width) 
      temp[1].x = imageSize.width - 1;
    if (temp[1].y >= imageSize.height) 
      temp[1].y = imageSize.height - 1;

    bool b = (temp != endpoints);
    if(b)
      DebugMessage("Endpoints out of range - fixed", 5);

    endpoints = temp;
    return b;
  }

//PartPolygons

  bool isPartPolygon(std::vector<cv::Point2f> partPolygon, float error)
  {
    bool b = (partPolygon.size() == 4);

    if (b)
    {
      cv::Point2f Z(0.0f, 0.0f);
      if(partPolygon[0] == Z && partPolygon[2] == Z)
        b = false;
      if (partPolygon[1] == Z && partPolygon[3] == Z)
        b = false;
    }

    if (b)
    {
      cv::Point2f d = partPolygon[3] - partPolygon[0];
      cv::Point2f f = partPolygon[2] - partPolygon[1];

      float D = d.x*d.x + d.y*d.y;
      float F = f.x*f.x + f.y*f.y;
      float e = D*error*error;

      b = (D > 0.0f) && (F > 0.0f);
      if(b) b = (abs(D - F) < e);
    }

    return b;
  }

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

  float getPartLenght(std::vector<cv::Point2f> partPolygon)
  {
    cv::Point2f d = partPolygon[1] - partPolygon[0];
    return sqrt(d.x*d.x + d.y*d.y);
  }

  float getPartWidth(std::vector<cv::Point2f> partPolygon)
  {
    cv::Point2f d = partPolygon[3] - partPolygon[0];
    return sqrt(d.x*d.x + d.y*d.y);
  }

  cv::Point2f getPartCenter(std::vector<cv::Point2f> partPolygon)
  {
    return 0.5f*(partPolygon[3] + partPolygon[1]);
  }

  cv::Point2f getCenter(std::vector<cv::Point2f> polygon)
  {
    cv::Point2f c(0.0f, 0.0f);

    if (polygon.size() > 0)
    {
      for (int i = 0; i < polygon.size(); i++)
        c += polygon[i];
      c = c / static_cast<float>(polygon.size());
    }

    return c;
  }

  std::map<int, std::vector<cv::Point2f>> getAllPolygons(Skeleton skeleton)
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

  std::vector<cv::Point2f> getEndpoints(std::vector<cv::Point2f> polygon)
  {
    cv::Point2f p0(FLT_MAX, FLT_MAX), p1(FLT_MIN, FLT_MIN);
    for(int i = 0; i < polygon.size(); i++)
    {
      if(polygon[i].x < p0.x) p0.x = polygon[i].x;  
      if(polygon[i].x > p1.x) p1.x = polygon[i].x;
      if(polygon[i].y < p0.y) p0.y = polygon[i].y;
      if(polygon[i].y > p1.y) p1.y = polygon[i].y;
    }
    std::vector<cv::Point2f> endpoints = { p0, p1 };

    return endpoints;
  }

  std::vector<cv::Point2f> getEndpoints(std::map<int, std::vector<cv::Point2f>> polygons)
  {
    std::vector<cv::Point2f> temp;
    for(int i = 0; i < polygons.size(); i++)
      for(int k = 0; k < polygons[i].size(); k++)
        temp.push_back(polygons[i][k]);

    std::vector<cv::Point2f> endpoints = getEndpoints(temp);
    temp.clear();

    return endpoints;
  }

  std::vector<cv::Point2f> getEndpoints(Skeleton skeleton)
  {
    std::map<int, std::vector<cv::Point2f>> polygons = getAllPolygons(skeleton);
    std::vector<cv::Point2f> endpoints = getEndpoints(polygons);
    polygons.clear();

    return endpoints;
  }

  std::map<uint32_t, std::vector<LimbLabel>> createLabels(Skeleton skeleton)
  {
    std::map<int, std::vector<cv::Point2f>> polygons = getAllPolygons(skeleton);
    std::map<uint32_t, std::vector<LimbLabel>> limbLabels;
    std::vector<Score> scores = { Score(0.0f, "keyframe") };
    std::vector<LimbLabel> temp;
    for (uint32_t i = 0; i < polygons.size(); i++)
    {
      LimbLabel label(int(i), getPartCenter(polygons[i]), 0.0f, polygons[i], scores);
      temp = { label };
      limbLabels.emplace(std::pair<uint32_t, std::vector<LimbLabel>>(i, temp));
      temp.clear();
    }
    scores.clear();
    polygons.clear();

    return limbLabels;
  }

  /*Solvlet createSolve(Skeleton & skeleton) // for testing
  {
    std::map<int, std::vector<cv::Point2f>> polygons = getAllPolygons(skeleton);
    std::vector<LimbLabel> limbLabels;
    std::vector<Score> scores;
    scores.push_back(Score(0.0f, "keyframe"));
    for (int i = 0; i < polygons.size(); i++)
      limbLabels.push_back(LimbLabel(i, getPartCenter(polygons[i]),0.0f, polygons[i], scores));

    Solvlet solve(0, limbLabels);
    polygons.clear();
    limbLabels.clear();
    scores.clear();

    return solve;
  }*/

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

  cv::Point2f SkeletonCenter(std::map<int, std::vector<cv::Point2f>> polygons)
  {
    cv::Point2f c = cv::Point2f(0.0f, 0.0f);
    float S = 0.0f;
    for (int k = 0; k < polygons.size(); k++)
    {
      float s = getPartLenght(polygons[k])*getPartWidth(polygons[k]);
      S += s;
      c = c + s*getPartCenter(polygons[k]);
    }

    if(S > 0.0f) 
      c = c/S;
    else
    {
      for(int i = 0; i < polygons.size(); i++)
        for(int k = 0; k < polygons[i].size(); k++)
        {
          S++;
          c += polygons[i][k];
        }
      if(S > 0.0f)
        c = c/S;
    }

    return c;
  }

  cv::Point2f SkeletonCenter(Skeleton skeleton)
  {
    std::map<int, std::vector<cv::Point2f>> polygons;
    polygons = getAllPolygons(skeleton);

    return SkeletonCenter(polygons);
  }

  void setSkeleton(Frame* frame, Frame* neighborFrame)
  {
    bool havePattern = (neighborFrame->getSkeleton().getPartTreePtr()->size() > 0);
    if (havePattern)
    {  
      // Create mask for the previous skeleton
      cv::Size size = frame->getMask().size();

      // Select mask ROI
      std::vector<cv::Point2i> endpoints1 = SearchROI(frame->getMask());
      cv::Rect ROI1 = toROIRect(endpoints1);
      correctROI(ROI1, size);

      // Selecting skeleton ROI
      Skeleton prevSkeleton = neighborFrame->getSkeleton();
      std::vector<cv::Point2f> endpoints2 = getEndpoints(prevSkeleton);
      cv::Rect ROI2 = toROIRect(endpoints2);
      correctROI(ROI2, size);

      // Calculate distance
      cv::Point2f shift(0, 0);
      cv::Point2i c1 = MaskCenter(frame->getMask(), ROI1);
      cv::Point2f c2 = SkeletonCenter(prevSkeleton);
      shift = cv::Point2i(static_cast<int>(c2.x), static_cast<int>(c2.y)) - c1;

      // Copy skeleton
      frame->setSkeleton(prevSkeleton - shift);
    }
  }

// Points
  bool inside(cv::Point2f p, cv::Size imageSize)
  {
    return inside(cv::Point2i(static_cast<int>(p.x), static_cast<int>(p.y)), imageSize);
  }

  bool inside(cv::Point p, cv::Size imageSize)
  {
    return (p.x >= 0 && p.x < imageSize.width && p.y >= 0 && p.y < imageSize.height);
  }

//Interpolation
  void clearSkeleton(Frame &frame)
  {
    if(frame.getFrametype() != KEYFRAME)
    {
      Skeleton empty;
      empty.setJointTree(tree<BodyJoint>());
      empty.setPartTree(tree<BodyPart>());

      frame.setSkeleton(empty);
    }
  }

  void clearSkeleton(Frame* frame)
  {
    if (frame->getFrametype() != KEYFRAME)
    {
      Skeleton empty;
      empty.setJointTree(tree<BodyJoint>());
      empty.setPartTree(tree<BodyPart>());

      frame->getSkeletonPtr()->getJointTreePtr()->clear();
      frame->getSkeletonPtr()->getPartTreePtr()->clear();
    }
  }

  void clearSkeletons(std::vector<Frame*> frames)
  {
    for(int i = 0; i < frames.size(); i++)
      clearSkeleton(frames[i]);
  }

  void interpolate(std::vector<Frame*> slice)
  {
    int n = slice.size();
    std::vector<int> interpolated;

    if(slice[0]->getFrametype() == KEYFRAME && slice[n - 1]->getFrametype() == KEYFRAME)
    {
      for (int i = 0; i < n; i++)
        if(slice[i]->getFrametype() == KEYFRAME)
          interpolated.push_back(i);
      while(interpolated.size() < n)
      {
        int m = interpolated.size() - 1;
        for(int t = 0; t < m; t++)
        {
          float f1 = interpolated[t];
          float f2 = interpolated[t+1];
          int k = static_cast<int>(0.5f*(f1 + f2));
          int N = slice[k]->getSkeleton().getJointTreePtr()->size();
          if (f1 != k && f2 != k && N == 0)
          {
            Skeleton S1 = slice[f1]->getSkeleton();
            Skeleton S2 = slice[f2]->getSkeleton();
            slice[k]->setSkeleton((S1 + S2)*0.5f);
            interpolated.push_back(k);
          }
        }
        std::sort(interpolated.begin(), interpolated.end());
      }
    }
  }

  void interpolate2(std::vector<Frame*> slice, bool useKeyframesOnly, bool replaceExisting)
  {
    int n = slice.size();

    std::vector<int> keyframes;
    for (int i = 0; i < n; i++)
    {
      if(slice[i]->getFrametype() == KEYFRAME)
        keyframes.push_back(i);
      else
        if(!useKeyframesOnly)
          if(slice[i]->getSkeletonPtr()->getPartTreePtr()->size() != 0)
            keyframes.push_back(i);
    }

    int m = keyframes.size() - 1;
    for(int t = 0; t < m; t++)
    {
      int f1 = keyframes[t];
      int f2 = keyframes[t+1];
      for(int k = f1 + 1; k != f2; k++)
      if (slice[k]->getFrametype() != KEYFRAME)
      if(slice[k]->getSkeletonPtr()->getPartTreePtr()->size() == 0 || replaceExisting)// ?
      {
        Skeleton S1 = slice[f1]->getSkeleton();
        Skeleton S2 = slice[f2]->getSkeleton();
        if (f2 - f1 > 0)
        {
          float K = static_cast<float>(k - f1)/static_cast<float>(f2 - f1);
          Skeleton skeleton = S1*(1.0f - K) + S2*K;
          skeleton.setName("interpolate2");
          
          slice[k]->setSkeleton(skeleton);
        }
      }
    }

  }

std::vector<int> interpolate3(std::vector<Frame*> frames, ImagePixelSimilarityMatrix* MSM, float SimilarityThreshold, bool replaceExisting)
  {
    float Q = SimilarityThreshold;
    if (Q <= 0.0f) Q = 0.55f;

    bool newISM = (MSM == 0);
    if(newISM) MSM = new ImagePixelSimilarityMatrix();
    if (MSM->size() != frames.size())
    {
      DebugMessage("ISM not found ", 2);
      DebugMessage("Building Image similarity matrix ", 2);
      MSM->buildImageSimilarityMatrix(frames, 0, 0, false, false);
      DebugMessage("ISM created ", 2);
      //MSM.write("MSM.txt");
    }

    std::vector<int> createdSkeletons;

    for (int i = 0; i < frames.size(); i++)
    {
      bool haveSkeleton = (frames[i]->getSkeleton().getPartTreeCount() > 0);
      if (frames[i]->getFrametype() != KEYFRAME && (!haveSkeleton || replaceExisting))
      {
        std::vector<Skeleton> S;
        std::vector<float> scores;
        for(int k = 0; k < frames.size(); k++)
        if(frames[k]->getFrametype() == KEYFRAME)
          if(MSM->at(i,k) > Q)
          {
            S.push_back((frames[k]->getSkeleton() - MSM->getShift(i,k)));
            scores.push_back(MSM->at(i, k));
          }
        Skeleton s0;
        s0.setJointTree(tree<BodyJoint>());
        s0.setPartTree(tree<BodyPart>());

        int n = scores.size();
        if(n > 0) 
        {
          float score = 0.0f;
          for(int k = 0; k < n; k++)
            score = score + scores[k];
          if(score > 0.0f)
            for (int k = 0; k < n; k++)
              scores[k] /= score;

          s0 = S[0]*0.0f;
          for (int k = 0; k < S.size(); k++)
          {
            Skeleton temp = s0 + S[k] * scores[k];
            s0 =  temp; // for supporting tree.hh 3.1
          }

          s0.setName("interpolate3");
          frames[i]->setSkeleton(s0);
          createdSkeletons.push_back(frames[i]->getID());
        }
        /*else frames[i]->setSkeleton(s0);*/       
      }

    }
    if (newISM) delete MSM;

    return createdSkeletons;
  }

  /*
  // For testing
  std::vector<int> interpolate4(std::vector<Frame*> frames, ImagePixelSimilarityMatrix* MSM, float SimilarityThreshold)
  {
    DebugMessage("Started nterpolate4 ", 2);
    interpolate2(frames);
    for (int i = 0; i < frames.size(); i++)
    {
      Skeleton skeleton = frames[i]->getSkeleton();
      if(skeleton.getName() == "interpolate2")
        if (skeletonScore(frames[i]->getMask(), skeleton) < SimilarityThreshold)
          clearSkeleton(frames[i]);
    }

    interpolate3(frames, MSM, SimilarityThreshold, false);
    for (int i = 0; i < frames.size(); i++)
    {
      Skeleton skeleton = frames[i]->getSkeleton();
      if(skeleton.getName() == "interpolate3")
        if (skeletonScore(frames[i]->getMask(), skeleton) < SimilarityThreshold)
          clearSkeleton(frames[i]);
    }
    propagateKeyFrames(frames, MSM, SimilarityThreshold, false);
    for (int i = 0; i < frames.size(); i++)
    {
      Skeleton skeleton = frames[i]->getSkeleton();
      if(skeleton.getName() == "propagateKeyFrames")
        if (skeletonScore(frames[i]->getMask(), skeleton) < SimilarityThreshold)
          clearSkeleton(frames[i]);
    }
    propagateFrames(frames, MSM, SimilarityThreshold, false);
    for (int i = 0; i < frames.size(); i++)
    {
      Skeleton skeleton = frames[i]->getSkeleton();
      if(skeleton.getName() == "propagateFrames")
        if (skeletonScore(frames[i]->getMask(), skeleton) < SimilarityThreshold)
          clearSkeleton(frames[i]);
    }
    interpolate2(frames, true, false);
    for (int i = 0; i < frames.size(); i++)
    {
      Skeleton skeleton = frames[i]->getSkeleton();
      if (skeleton.getName() == "interpolate2")
        if (skeletonScore(frames[i]->getMask(), skeleton) < SimilarityThreshold)
          clearSkeleton(frames[i]);
    }
    DebugMessage("Interpolate4 finished ", 2);

    return std::vector<int>();
  }*/

  std::vector<int> propagateKeyFrames(std::vector<Frame*> frames, ImagePixelSimilarityMatrix* MSM, float SimilarityThreshold, bool replaceExisting)
  {
    float Q = SimilarityThreshold;
    if(Q <= 0.0f) Q = 0.55f;

    bool newISM = (MSM == 0);
    if (newISM) MSM = new ImagePixelSimilarityMatrix();
    if (MSM->size() != frames.size())
    {
      DebugMessage("ISM not found ", 2);
      DebugMessage("Building Image similarity matrix ", 2);
      MSM->buildImageSimilarityMatrix(frames, 0, 0, false, false);
      DebugMessage("ISM created ", 2);
      //MSM.write("MSM.txt");
    }

    std::vector<int> Replaced;

    for (int i = 0; i < frames.size(); i++)
    {
      bool haveSkeleton = (frames[i]->getSkeletonPtr()->getPartTreePtr()->size() > 0);
      if (frames[i]->getFrametype() != KEYFRAME && (!haveSkeleton || replaceExisting))
      {
        int n = -1;
        float f = 0.0f;
        for (int k = 0; k < frames.size(); k++)
        {
          if(frames[k]->getFrametype() == KEYFRAME)
          {
            if(MSM->at(i,k) > Q && f < MSM->at(i,k) )
            {
              n = k;
              f = MSM->at(i, k);
            }
          }
        }
        //std::cout << "frame [" << i << "] skeleton similar to " << n << std::endl;
  
        if(n > -1) 
        {
          Skeleton skeleton = frames[n]->getSkeleton() - MSM->getShift(i, n);
          skeleton.setName("propagateKeyFrames");
          frames[i]->setSkeleton(skeleton);
          Replaced.push_back(frames[i]->getID());
        }
      }
    }
    if (newISM) delete MSM;

    return Replaced;
  }

  std::vector<int> propagateFrames(std::vector<Frame*> frames, ImagePixelSimilarityMatrix* MSM, float SimilarityThreshold, bool replaceExisting)
  {
    float Q = SimilarityThreshold;
    if(Q <= 0.0f) Q = 0.55f;

    bool newISM = (MSM == 0);
    if (newISM) MSM = new ImagePixelSimilarityMatrix();
    if (MSM->size() != frames.size())
    {
      DebugMessage("ISM not found ", 2);
      DebugMessage("Building Image similarity matrix ", 2);
      MSM->buildImageSimilarityMatrix(frames, 0, 0, false, false);
      DebugMessage("ISM created ", 2);
      //MSM.write("MSM.txt");
    }

    std::vector<int> Replaced;

    for (int i = 0; i < frames.size(); i++)
    {
      bool emptySkeleton = frames[i]->getSkeletonPtr()->getPartTreePtr()->size() == 0;
      if ((frames[i]->getFrametype() != KEYFRAME) && (emptySkeleton || replaceExisting))
      {
        int n = -1;
        float f = 0.0f;
        for (int k = 0; k < frames.size(); k++)
          if(frames[k]->getSkeletonPtr()->getPartTreePtr()->size() != 0)
          {
            if(MSM->at(i,k) > Q && f < MSM->at(i,k) )
            {
              n = k;
              f = MSM->at(i, k);
            }
          }

        DebugMessage("frame [" + std::to_string(i) +  "] skeleton similar to " + std::to_string(n), 2);
 
        if(n > -1)
        {
          Skeleton skeleton = frames[n]->getSkeleton() - MSM->getShift(i, n);
          skeleton.setName("propagateFrames");
          frames[i]->setSkeleton(skeleton);
          Replaced.push_back(frames[i]->getID());
        }
      }
    }
    if (newISM) delete MSM;

    return Replaced;
  }

  /*
  float skeletonScore(cv::Mat mask, Skeleton skeleton, cv::Point2f skeletonShift)
  {
    float score = -1.0f;
    if(skeleton.getPartTreePtr()->size() != 0)
    {
      // Create part polygons
      Skeleton tempSkeleton = skeleton + skeletonShift;
      std::map<int, std::vector<cv::Point2f>> polygons = getAllPolygons(tempSkeleton);

      //Select union ROI
      std::vector<cv::Point2i> maskEndpoints = SearchROI(mask);
      std::vector<cv::Point2f> skeletonEndpoints = getEndpoints(polygons);
      for(int i = 0; i < maskEndpoints.size(); i++)
        skeletonEndpoints.push_back(static_cast<cv::Point2f>(maskEndpoints[i]));
      skeletonEndpoints = getEndpoints(skeletonEndpoints);
      cv::Point2i p0 = skeletonEndpoints[0];
      cv::Point2i p1 = skeletonEndpoints[1];

      //Create skeleton mask
      cv::Mat skeletonMask = cv::Mat::zeros(mask.size(), CV_8UC1);
      putSkeletonMask(skeletonMask, tempSkeleton);

      // Compare
      int Q = 9;
      
      int intersection = 0, difference = 0;
      for(int x = p0.x; x < p1.x; x++)
        for(int y = p0.y; y < p1.y; y++)
        {
          bool color1 = false;
          bool color2 = false;
          cv::Point2i p = cv::Point2i(x, y);
          if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows)
            color1 = (mask.at<uchar>(y, x) > Q);
          if (p.x >= 0 && p.x < mask.cols && p.y >= 0 && p.y < mask.rows)
            color2 = (skeletonMask.at<uchar>(p.y, p.x) > Q);
          if (color1 && color2) intersection++;
          if (color1 != color2) difference++;
        }
      if(difference + intersection > 0)
        score = static_cast<float>(intersection) / static_cast<float>(difference + intersection);

      skeletonMask.release();
      skeletonEndpoints.clear();
      maskEndpoints.clear();
      polygons.clear();
    }

    return score;
  }*/

//Visualization

  void putPartRect(cv::Mat image, std::vector<cv::Point2f> polygon, cv::Scalar color)
  {
    polygon.push_back(polygon[0]);
    for (unsigned int i = 1; i < polygon.size(); i++)
      line(image, polygon[i - 1], polygon[i], color, 1, 1);
  }

  void putSkeleton(cv::Mat image, Skeleton skeleton, cv::Scalar color)
  {
    std::map<int, std::vector<cv::Point2f>> polygons = getAllPolygons(skeleton);
    for (auto p = 0; p < polygons.size(); p++)
      putPartRect(image, polygons[p], color);
  }

  void putSkeletonMask(cv::Mat &mask, Skeleton skeleton, cv::Size maskSize, uchar color)
  {
    std::map<int, std::vector<cv::Point2f>> polygons = getAllPolygons(skeleton);
    std::vector<cv::Point2f> endpoints = getEndpoints(polygons);
    cv::Rect ROI = toROIRect(endpoints);

    //Creat mew mask image if maskSize > size(0, 0)
    if (maskSize.width > 0 && maskSize.height > 0)
    {
      mask.release();
      mask = cv::Mat::zeros(maskSize, CV_8UC1);
    }
    //Creat mew mask image if the mask size is zero
    if (mask.size().height == 0 || mask.size().width == 0)
    {
      mask.release();
      mask = cv::Mat::zeros(ROI.size(), CV_8UC1);
      polygons = getAllPolygons(skeleton - endpoints[0]);
    }

    //draw skeleton mask
    bool outOfRange = false;
    cv::Size size = mask.size();
    for(int i = 0; i < polygons.size(); i++)
    {
      std::vector<cv::Point2f> endpoints = getEndpoints(polygons[i]);
      outOfRange = correctEndpoints(endpoints, size);
      for(float x = endpoints[0].x; x <= endpoints[1].x; x++)
        for (float y = endpoints[0].y; y <= endpoints[1].y; y++)
        {
          cv::Point2f p(x, y);
          if(cv::pointPolygonTest(polygons[i], p, false) > 0)
          {
            //if (inside(p, size))
              mask.at<uchar>(y, x) = color;
            /*else
              outOfRange = true;*/       
          }		  
        }

      if (outOfRange)
        DebugMessage("Polygon " + std::to_string(i) + " mask out of range", 2);
    }	 

    polygons.clear();
    endpoints.clear();
  }

  void putLabels(cv::Mat image, std::vector<LimbLabel> frameLabels, cv::Scalar color)
  {
    for (int k = 0; k < frameLabels.size(); k++)
      putPartRect(image, frameLabels[k].getPolygon(), color);
  }

  std::string to_string(int num, uchar length)
  {
    std::string s = std::to_string(num);
    while(s.length() < length)
      s = "0" + s;

    return s;
  }

  std::map<uint32_t, std::vector<LimbLabel>> generateLimbLabels(Frame* frame, std::map <std::string, float> params)
  {
    //Emplace default parameters
    params.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT());
    params.emplace(DETECTOR_DETECT_PARAMETERS::MIN_THETA());
    params.emplace(DETECTOR_DETECT_PARAMETERS::MAX_THETA());
    params.emplace(DETECTOR_DETECT_PARAMETERS::STEP_THETA());
    params.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT());

    //Set used parameters values
    const int Q = 10; // black pixel threshold
    auto searchDistCoeff = params.at(
      DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT().name());
    auto minTheta = params.at(DETECTOR_DETECT_PARAMETERS::MIN_THETA().name());
    auto maxTheta = params.at(DETECTOR_DETECT_PARAMETERS::MAX_THETA().name());
    auto stepTheta = params.at(DETECTOR_DETECT_PARAMETERS::STEP_THETA().name());
    auto searchStepCoeff = params.at(
      DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT().name());

    auto skeleton = frame->getSkeleton();
    //skeleton.resize(resizeFactor); // << !!!
    auto partPolygons = getAllPolygons(skeleton);
    auto partTree = skeleton.getPartTree();

    //Creating limb labels
    std::map<uint32_t, std::vector<LimbLabel>> Labels;
    for (auto p = partTree.begin(); p != partTree.end(); p++)
    {
      auto partID = p->getPartID();
      std::vector<cv::Point2f> partPolygon = partPolygons[partID];
      auto partLenght = getPartLenght(partPolygon);
      auto partWidth = getPartWidth(partPolygon);
      auto partCenter = getPartCenter(partPolygon);
      auto partAngle = static_cast<float>(spelHelper::getAngle(partPolygon[0], partPolygon[1]));
      std::vector<cv::Point2f> LabelPolygon(4), RotatedPolygon(4);

      auto minStep = abs(searchStepCoeff*partWidth);
      minStep = std::max(minStep, 2.0f);

      auto searchDist = p->getSearchRadius();
      if(searchDist <= 0)
        searchDist = searchDistCoeff*partLenght;
      if (searchDist <= 0)
        searchDist = minStep + 1;

      auto mask = frame->getMask();
      auto maskSize = mask.size();
      std::vector<LimbLabel> partLabels;
      auto deltaTheta = std::abs(p->getRotationSearchRange());
      auto maxLocalTheta = p->getRotationSearchRange() == 0 ?
        maxTheta : deltaTheta;
      auto minLocalTheta = p->getRotationSearchRange() == 0 ?
        minTheta : deltaTheta;
      //Iterations by angle
      for (float angleShift = -minLocalTheta; angleShift < maxLocalTheta; angleShift += stepTheta)
      {        
        float labelAngle = partAngle + angleShift;
        for (int t = 0; t < partPolygon.size(); t++)
          RotatedPolygon[t] = spelHelper::rotatePoint2D(partPolygon[t], partCenter, angleShift);//Part polygon rotation
        //Iterations by coordinates
        for (float x = -searchDist; x < searchDist; x += minStep)
          for (float y = -searchDist; y < searchDist; y += minStep)
          {
            auto labelCenter = partCenter + cv::Point2f(x, y);//Shift label center
            auto blackPixel = true;
            if(inside(partCenter, maskSize))
              blackPixel = (mask.at<uint8_t>(static_cast<int>(labelCenter.y), static_cast<int>(labelCenter.x)) <= Q);
            if (!blackPixel)
            {		  
              for (int t = 0; t < partPolygon.size(); t++)
                LabelPolygon[t] = RotatedPolygon[t] + cv::Point2f(x, y);//Shift label polygon			  
              partLabels.push_back(LimbLabel(partID, labelCenter, labelAngle, LabelPolygon, std::vector<Score>(), false));//Create LimbLabel
            }         
          } 
      }
      //Reinsurance: if part labels is empty -
      // creating labels only by angle around part center, independently the pixel color  //??
      if (partLabels.size() == 0)
        for (auto angleShift = -minLocalTheta; angleShift < maxLocalTheta; angleShift += stepTheta) 
        {
          auto labelAngle = partAngle + angleShift;
          for (int t = 0; t < partPolygon.size(); t++)
            RotatedPolygon[t] = spelHelper::rotatePoint2D(partPolygon[t], partCenter, angleShift);//Part polygon rotation
          partLabels.push_back(LimbLabel(partID, partCenter, labelAngle, LabelPolygon, std::vector<Score>(), false));//Create LimbLabel
        }
      Labels.emplace(std::pair<uint32_t, std::vector<LimbLabel>>(partID, partLabels));
      partLabels.clear();
    }

    return Labels;
  }
}