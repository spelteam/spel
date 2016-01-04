// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include "TestsFunctions.hpp"
  
namespace SPEL
{  

  void PutPartRect(Mat &Image, vector<Point2f> polygon, Scalar color)
  {
    polygon.push_back(polygon[0]);
    for (unsigned int i = 1; i < polygon.size(); i++)
      line(Image, polygon[i - 1], polygon[i], color, 1, 1);
  }

  //Build bodypart rectangle on bodypart joints
  POSERECT<Point2f> BuildPartRect(BodyJoint *j0, BodyJoint *j1, float LWRatio)
  {
    Point2f p0 = j0->getImageLocation(), p1 = j1->getImageLocation();
    float boneLength = (float)sqrt(spelHelper::distSquared(p0, p1)); // distance between nodes
    float boneWidth = boneLength / LWRatio;
    Point2f boxCenter = p0 * 0.5 + p1 * 0.5; // the bobypart center  coordinates
    // Coordinates for drawing of the polygon at the coordinate origin
    Point2f c1 = Point2f(0.f, 0.5f * boneWidth);
    Point2f c2 = Point2f(boneLength, 0.5f * boneWidth);
    Point2f c3 = Point2f(boneLength, -0.5f * boneWidth);
    Point2f c4 = Point2f(0.f, -0.5f * boneWidth);
    Point2f polyCenter = Point2f(boneLength * 0.5f, 0.f); // polygon center 
    Point2f direction = p1 - p0; // used as estimation of the vector's direction
    float rotationAngle = float(spelHelper::angle2D(1.0f, 0.0f, direction.x, direction.y) * (180.0 / M_PI)); //bodypart tilt angle 
    // Rotate and shift the polygon to the bodypart center
    c1 = spelHelper::rotatePoint2D(c1, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c2 = spelHelper::rotatePoint2D(c2, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c3 = spelHelper::rotatePoint2D(c3, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c4 = spelHelper::rotatePoint2D(c4, polyCenter, rotationAngle) + boxCenter - polyCenter;
    POSERECT <Point2f> poserect(c1, c2, c3, c4);
    return poserect;
  }

  //Build the rectangles for all of bodyparts
  map<int, POSERECT<Point2f>> SkeletonRects(Skeleton skeleton)
  {
    map<int, POSERECT<Point2f>> Rects;
    tree<BodyPart> PartTree = skeleton.getPartTree();
    for (tree<BodyPart>::iterator BP_iterator = PartTree.begin(); BP_iterator != PartTree.end(); BP_iterator++)
    {
      BodyJoint *j0 = skeleton.getBodyJoint(BP_iterator->getParentJoint());
      BodyJoint *j1 = skeleton.getBodyJoint(BP_iterator->getChildJoint());
      POSERECT<Point2f> Rect = BuildPartRect(j0, j1, BP_iterator->getLWRatio());
      Rects.emplace(pair<int, POSERECT<Point2f>>((*BP_iterator).getPartID(), Rect));
    }
    return Rects;
  }

  // Selecting locations of all body part from skeleton
  map<int, pair<Point2f, Point2f>> getPartLocations(Skeleton skeleton)
  {
    map<int, pair<Point2f, Point2f>> PartLocations;
    BodyJoint* J0, *J1;
    Point2f p0, p1;
    tree <BodyPart> partTree = skeleton.getPartTree();
    for (tree <BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
    {
      J0 = skeleton.getBodyJoint(i->getParentJoint());
      J1 = skeleton.getBodyJoint(i->getChildJoint());
      p0 = J0->getImageLocation();
      p1 = J1->getImageLocation();
      PartLocations.emplace(pair<int, pair<Point2f, Point2f>>(i->getPartID(), pair<Point2f, Point2f>(p0, p1)));
    }
    return PartLocations;
  }

  //Returns "true" if polygon is crossed (occluded) by "rect"
  bool IsCrossed(vector<Point2f> PolygonPoints, POSERECT<Point2f> rect)
  {
    bool Crossed = 0;
    for (unsigned int i = 0; i < PolygonPoints.size() - 1; i++)
    {
      unsigned int k = i;
      if (k == PolygonPoints.size() - 1)
        k = 0;
      Point2f delta = PolygonPoints[i + 1] - PolygonPoints[k];
      float n = 2 * max(abs(delta.x), abs(delta.y));
      float dx = delta.x / n;
      float dy = delta.y / n;
      float x = PolygonPoints[i].x, y = PolygonPoints[i].y;
      for (int m = 0; m < n; m++)
      {
        Crossed = Crossed || (rect.containsPoint(Point2f(x, y)) > 0);
        y = y + dy;
        x = x + dx;
      }
    }
    return Crossed;
  }

  //Returns "true" if "rect1" is crossed (occluded) by "rect2"
  bool IsCrossed(POSERECT<Point2f> rect1, POSERECT<Point2f> rect2)
  {
    vector<Point2f> Polygon = rect1.asVector();
    bool Crossed = IsCrossed(Polygon, rect2);
    Polygon.clear();
    return Crossed;
  }

  // For calculation of polygons depth priority
  class depthCompare
  {
  public:
      bool operator () (pair<int, int> X, pair<int, int> Y)
      {
          return Y.second > X.second;
      }
  };

  //For the each polygon select all polygons, which crossed it 
  vector<vector<pair<int, int>>> CrossingsList(map<int, POSERECT<Point2f>> Rects, map<int, int> depth)
  {
    vector<vector<pair<int, int>>> Crosses;
    for (unsigned int i = 0; i < Rects.size(); i++)
    {
      vector<pair<int, int>> X;
      vector<Point2f> Polygon = Rects[i].asVector();
      for (unsigned int k = 0; k < Rects.size(); k++)
        if (depth[k] < depth[i])
        {
          if (IsCrossed(Polygon, Rects[k]))
            X.push_back(pair<int, int>(k, depth[k]));
        }
      sort(X.begin(), X.end(), depthCompare());
      Crosses.push_back(X);
      X.clear();
      Polygon.clear();
    }
    return Crosses;
  }

  //Build set of the rect pixels colours 
  vector <Point3i> GetPartColors(Mat image, Mat mask, POSERECT < Point2f > rect)
  {
    vector <Point3i> PartColors;
    int xmin, ymin, xmax, ymax;
    rect.GetMinMaxXY <int>(xmin, ymin, xmax, ymax);
    for (int x = xmin; x < xmax; x++)
    {
      for (int y = ymin; y < ymax; y++)
      {
        if ((rect.containsPoint(Point2f(float(x), float(y))) > 0) && (mask.at<uint8_t>(y, x) > 9))
        {
          Vec3b color = image.at<Vec3b>(y, x);
          PartColors.push_back(Point3i(color[0], color[1], color[2]));
        }
      }
    }
    return PartColors;
  }

  //Normalization of the  histogram
  void  Normalize(vector <vector <vector <float>>> &Histogramm, int nBins, int pixelsCount)
  {
    for (int r = 0; r < nBins; r++)
      for (int g = 0; g < nBins; g++)
        for (int b = 0; b < nBins; b++)
          Histogramm[b][g][r] = Histogramm[b][g][r] / pixelsCount;
  }

  //Output histogram into text file
  void PutHistogram(ofstream &fout, vector <vector <vector <float>>> &Histogramm, int sizeFG)
  {
    int nBins = 8;
    for (int r = 0; r < nBins; r++)
      for (int g = 0; g < nBins; g++)
        for (int b = 0; b < nBins; b++)
          if (Histogramm[b][g][r]>0)
            fout << "Histogram[" << r << "," << g << "," << b << "] = " << Histogramm[b][g][r] * sizeFG << ";\n";
  }

  //Loading frames from project
  vector<Frame*> LoadTestProject(string FilePath, string FileName)
  {
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif
    ProjectLoader projectLoader(FilePath);
    projectLoader.Load(FilePath + FileName);
    vector<Frame*> frames, temp = projectLoader.getFrames();
    for (unsigned int i = 0; i < temp.size(); i++)
    {
      frames.push_back(new Frame(temp[i]->getFrametype()));
      temp[i]->clone(frames[i]);
    }
    return frames;
  }

  /*
  //Set parameters from the frames sequence
  map <string, float> SetParams(vector<Frame*> frames, Sequence **seq)
  {
    //This fragment produces crash with message: "The program has exited with code 3 (0x3)."
    map <string, float> params;
    *seq = new Sequence(0, "colorHistDetector", frames);
    if (*seq != 0)
    {
      (*seq)->estimateUniformScale(params);
      (*seq)->computeInterpolation(params);
    }
    return params;
  }*/

  //Loading frames from project and set "params"
  vector<Frame*> LoadTestProject(map <string, float> &params, string FilePath, string FileName)
  {

#if defined(WINDOWS) && defined(_MSC_VER)
      if (IsDebuggerPresent())
          FilePath = "Debug/" + FilePath;
#endif
      ProjectLoader projectLoader(FilePath);
      projectLoader.Load(FilePath + FileName);
      vector<Frame*> frames, temp = projectLoader.getFrames();

      for (unsigned int i = 0; i < temp.size(); i++)
      {
        frames.push_back(new Frame(temp[i]->getFrametype()));
        temp[i]->clone(frames[i]);
      }

      Sequence *seq = new Sequence(0, "colorHistDetector", frames);
      if (seq != 0)
      {
        seq->estimateUniformScale(params);
        seq->computeInterpolation(params);
      }
      for (auto f : frames)
          delete f;
      frames.clear();
      frames = seq->getFrames();

      return frames;
  }


  //Counting of keyframes in set of frames 
  int keyFramesCount(vector<Frame*> frames)
  {
    int KeyframesCount = 0;
    for (unsigned int i = 0; i < frames.size(); i++)
      if (frames[i]->getFrametype() == KEYFRAME)
        KeyframesCount++;
    return KeyframesCount;
  }

  //Returns  index of first keyframe
  int FirstKeyFrameNum(vector<Frame*> frames)
  {
	unsigned int i = 0;
	int FirstKeyframe = -1;
    while ((i < frames.size()) && (FirstKeyframe == -1))
    {
      if (frames[i]->getFrametype() == KEYFRAME)
        FirstKeyframe = i;
      i++;
    }
    return FirstKeyframe;
  }

  vector<Point2f> getPartRect(float LWRatio, Point2f p0, Point2f p1) // == DetectorGetPartRect, used another way
  {
    vector<Point2f> partRect;
    Point2f d = p0 - p1;
    //float L = sqrt(pow(p1.x - p0.x,2) + pow(p1.y - p0.y, 2));
    float k = 0.5f/LWRatio;
    float dx = k*(d.y);
    float dy = k*(-d.x);
    partRect.push_back(Point2f(p0.x + dx, p0.y + dy));
    partRect.push_back(Point2f(p1.x + dx, p1.y + dy));
    partRect.push_back(Point2f(p1.x - dx, p1.y - dy));
    partRect.push_back(Point2f(p0.x - dx, p0.y - dy));

    return partRect;
  }

  vector<Point2f> getPartRect(float LWRatio, Point2f p0, Point2f p1, cv::Size blockSize) // == DetectorGetPartRect, used another way
  {
    Point2f d = p0 - p1;
    //d.x = -d.x;
    Point2f center = 0.5*(p0 + p1);
    float L = sqrt(pow(d.x, 2) + pow(d.y, 2));
    float L1 = 0;
    if (blockSize.width > 0)
      L1 = blockSize.width * ceil(L / blockSize.width);
    float c = L1 / L;
    Point2f d_ = c*d;
    //Point2f p0_ = center - 0.5*d_;
    //Point2f p1_ = center + 0.5*d_;
    Point2f p0_ = p0 + 0.5*(d_ - d);
    Point2f p1_ = p1 - 0.5*(d_ - d);
    L = L1; //more logically without this line
    float W = L / LWRatio;
    if (blockSize.height > 0)
      W = blockSize.height*ceil(W / blockSize.height);
    float k = 0.5f*W / L;
    float dx = k*(d_.y);
    float dy = k*(-d_.x);
    vector<Point2f> partRect;
    partRect.push_back(Point2f(p0_.x + dx, p0_.y + dy));
    partRect.push_back(Point2f(p1_.x + dx, p1_.y + dy));
    partRect.push_back(Point2f(p1_.x - dx, p1_.y - dy));
    partRect.push_back(Point2f(p0_.x - dx, p0_.y - dy));

    return partRect;
  }

  void CompareSolves(vector<Solvlet> Solves, vector<Frame*> Frames, ImageSimilarityMatrix &ISM)
  {
    ASSERT_GE(Solves.size(), 0);

    // Copy ID of all frames, which identical to the selected frame ("frameID")
    map<int, vector<int>> IdenticalFrames;
    for (unsigned int i = 0; i < ISM.size(); i++)
      for (unsigned int k = 0; k < ISM.size(); k++)
      {
        vector<int> temp;
        if((Frames[i]->getFrametype() == KEYFRAME))
          if ((ISM.at(i, k) <= 0.05) && (Frames[k]->getFrametype() != KEYFRAME))
            temp.push_back(k);
        IdenticalFrames.emplace(pair<int, vector<int>>(i,temp));
        temp.clear();
      }// IdenticalFrames[i] consist ID of all lockframes, wich identical to Frames[i]  

      // Compare parts from all identical frames
      float AcceptableError = 2; // 2 pixels
      for (unsigned int i = 0; i < IdenticalFrames.size(); i++)
      {
        Skeleton skeleton = Frames[i]->getSkeleton();
        map<int, pair<Point2f, Point2f>> PartLocations = getPartLocations(skeleton);
        for (unsigned int n = 0; n < IdenticalFrames[i].size(); n++)
          for (unsigned int k = 0; k < Solves.size(); k++)
            if (Solves[k].getFrameID() == IdenticalFrames[i][n])
              {
                vector<LimbLabel> Labels = Solves[k].getLabels();
                //Compare all parts from current lockframe and root frame
                for (unsigned int t = 0; t < Labels.size(); t++)
                 {
                   Point2f l0, l1; // Actual current part joints locations
                   Labels[t].getEndpoints(l0, l1);
                   int partID = Labels[t].getLimbID();

                   Point2f p0, p1; // Expected current part joints locations
                   p0 = PartLocations[partID].first;// [t]
                   p1 = PartLocations[partID].second;// [t]

                   Point2f delta0 = l0 - p0;
                   Point2f  delta1 = l1 - p1;
                   float error_A = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
                   delta0 = l0 - p1;
                   delta1 = l1 - p0;
                   float error_B = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
                   float error = min(error_A, error_B); // Distance between ideal body part and label
                   EXPECT_LE(error, AcceptableError) << "RootFrameID = " << i << ", LockframeID = " << IdenticalFrames[i][n] << ", PartID = " << partID << ", Error = " << error << "(pixels)" << endl;
                }
              }
        }
  }

// "TestISM" class
TestISM::TestISM(void)
  {
    id = 0x4950534D;
  }
// Empty functions
TestISM::TestISM(const TestISM & m): ImageSimilarityMatrix(m){}
//TestISM::TestISM(const std::vector<Frame*>& frames) : TestISM(){}
TestISM::TestISM(TestISM && m): ImageSimilarityMatrix(std::move(m)){}
TestISM::~TestISM(void){}
TestISM & TestISM::operator=(const TestISM & s)
{
  return *this;
}
void TestISM::computeISMcell(const Frame* left, const Frame* right, const int maxFrameHeight){}

// The smooth function
 float F(float x)
 {
   return 1.0f - exp(-0.5f*x);
 }

 // Building normalized ISM of the frames sequence
 // "useOverlapFactor" chooses gradual ("true") or abrupt("false") increase assesment of the mask mismatch
 void TestISM::build(vector<Frame*> frames, bool useOverlapFactor = false)
 {
   int n = frames.size();
   float max_ = 0;
   Point2i N;
   const int maxColorDist = static_cast<int>(3 * pow(255, 2));
   imageSimilarityMatrix = Mat::zeros(Size(n, n), cv::DataType<float>::type);
   imageShiftMatrix = Mat::zeros(Size(n, n), cv::DataType<cv::Point2f>::type);
   
   for (int i = 0; i < n; i++)
     for (int k = 0; k < i; k++)
     {
       Mat I1 = frames[i]->getImage();
       Mat M1 = frames[i]->getMask();
       Mat I2 = frames[k]->getImage();
       Mat M2 = frames[k]->getMask();

       Point2f M1_center = Point2f(0, 0);
       Point2f M2_center = Point2f(0, 0);
       int M1_area = 0, M2_area = 0, M_area = 0;

       int rows = std::min(M1.rows, M2.rows);
       int cols = std::min(M1.cols, M2.cols);

       Point2f M1_min = Point2f(1.0e+7, 1.0e+7), M1_max = Point2f(0, 0);
       Point2f M2_min = Point2f(1.0e+7, 1.0e+7), M2_max = Point2f(0, 0);

       // calculation the masks area
       for (int y = 0; y < rows; y++)
         for (int x = 0; x < cols; x++)
         {
           if (M1.at<uchar>(y, x) >= 10)
           {
             M1_area++;
             M1_center += Point2f(float(x), float(y));

             if (M1_min.x > float(x)) M1_min.x = float(x);
             if (M1_min.y > float(y)) M1_min.y = float(y);
             if (M1_max.x < float(x)) M1_max.x = float(x);
             if (M1_max.y < float(y)) M1_max.y = float(y);
           }
           if (M2.at<uchar>(y, x) >= 10)
           {
             M2_area++;
             M2_center += Point2f(float(x), float(y));
             
             if (M2_min.x > float(x)) M2_min.x = float(x);
             if (M2_min.y > float(y)) M2_min.y = float(y);
             if (M2_max.x < float(x)) M2_max.x = float(x);
             if (M2_max.y < float(y)) M2_max.y = float(y);
           }
         }

       // calculation the shifts and normalized "frames similarity scores"
       Point2f shift(float(2*cols), float(2*rows)); // default value: infinity - mask outside the frame
       float CellScore = 0.0f;

       if ((M1_area > 0) && (M2_area > 0))
       { 
         M1_center = M1_center*(1.0f / M1_area);
         M2_center = M2_center*(1.0f / M2_area);
         shift = M2_center - M1_center;

         M1_min.x = std::max(M1_min.x, M2_min.x - shift.x);
         M1_min.y = std::max(M1_min.y, M2_min.y - shift.y);
         M1_max.x = std::min(M1_max.x, M2_max.x - shift.x);
         M1_max.y = std::min(M1_max.y, M2_max.y - shift.y);

         M_area = 0;
         for (int x = (int)M1_min.x; x < (int)M1_max.x; x++)
           for (int y = (int)M1_min.y; y < (int)M1_max.y; y++)
           {			 
             Point2f A = Point2f(float(x), float(y));
             Point2f B = A + shift;
             float color_squareDistance = 0; // for background pixel 

             if ((B.x >= 0) && (B.x < cols) && (B.y >= 0) && (B.y < rows))
             {
               if ((M1.at<uchar>(y, x) > 9) && (M2.at<uchar>(static_cast<int>(B.y), static_cast<int>(B.x)) > 9))
               {
                 M_area++; // overlapped area
                 Vec3b A_color = I1.at<Vec3b>(y, x);
                 Vec3b B_color = I2.at<Vec3b>(y, x);
                 color_squareDistance = 0;
                 for (int q = 0; q < 3; q++)
                   color_squareDistance += static_cast<float>(pow(B_color[q] - A_color[q], 2));
               }
               CellScore += color_squareDistance / maxColorDist;
             }
           }
         float composite_area = static_cast<float>(M1_area + M2_area - M_area);

         float mulct = 1.0f - static_cast<float>(M_area) / composite_area; // masks overlap factor
         if (useOverlapFactor) mulct = F(100 * mulct); // initialized for non-overlapping pixels

         //cout <<"M1_area = " << M1_area << ", M2_area = " << M2_area << ", M_area = " << M_area << ", mulct = " << mulct << endl;
         CellScore = mulct + CellScore / composite_area; // == Summ(PixelScores)/Summ(MaxPixelScore) == (different_mask_area*maxColorDist + Summ(intersecrt_mask_area_ColorDistances)) / (maxColorDist*composite_area), in [0..1]
       }
       if ((M1_area <= 0) || (M2_area <= 0))
         CellScore = 1.0f;
       imageShiftMatrix.at<Point2f>(i, k) = shift;
       imageShiftMatrix.at<Point2f>(k, i) = -shift;
       imageSimilarityMatrix.at<float>(i, k) = CellScore;
       imageSimilarityMatrix.at<float>(k, i) = CellScore;

       // Temporary debug info
       if (CellScore > max_)
       {
         max_ = CellScore;
         N = Point2i(i, k);
       }
       //

       I1.release();
       M1.release();
       I2.release();
       M2.release();
     }

   // Temporary debug info
   cout << "Max score = " << max_ << ", Frames =" << N << endl;
   //

 }  
}