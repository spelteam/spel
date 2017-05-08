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
  spelRECT<Point2f> BuildPartRect(BodyJoint *j0, BodyJoint *j1, float LWRatio)
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
    spelRECT <Point2f> poserect(c1, c2, c3, c4);
    return poserect;
  }

  //Build the rectangles for all of bodyparts
  map<int, spelRECT<Point2f>> SkeletonRects(Skeleton skeleton)
  {
    map<int, spelRECT<Point2f>> Rects;
    tree<BodyPart> PartTree = skeleton.getPartTree();
    for (tree<BodyPart>::iterator BP_iterator = PartTree.begin(); BP_iterator != PartTree.end(); BP_iterator++)
    {
      BodyJoint *j0 = skeleton.getBodyJoint(BP_iterator->getParentJoint());
      BodyJoint *j1 = skeleton.getBodyJoint(BP_iterator->getChildJoint());
      spelRECT<Point2f> Rect = BuildPartRect(j0, j1, BP_iterator->getLWRatio());
      Rects.emplace(pair<int, spelRECT<Point2f>>((*BP_iterator).getPartID(), Rect));
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
  bool IsCrossed(vector<Point2f> PolygonPoints, spelRECT<Point2f> rect)
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
  bool IsCrossed(spelRECT<Point2f> rect1, spelRECT<Point2f> rect2)
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
  vector<vector<pair<int, int>>> CrossingsList(map<int, spelRECT<Point2f>> Rects, map<int, int> depth)
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
  vector <Point3i> GetPartColors(Mat image, Mat mask, spelRECT < Point2f > rect)
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
//---------------------------------------------------------------------------------------
//TestProjectLoader
  TestProjectLoader::TestProjectLoader()
  {
    SetCurFolder("");
  }

  TestProjectLoader::TestProjectLoader(string FilePath, string FileName)
  {
    TestProjectLoader::Load(FilePath, FileName);
  }

  bool TestProjectLoader::Load(string FilePath, string FileName)
  {
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif
    SetCurFolder(FilePath);
    bool loaded = ProjectLoader::Load(FilePath + FileName);
    return loaded;
  }
//---------------------------------------------------------------------------------------
//TestSequence
  bool TestSequence::Load(string FilePath, string FileName)
  {
    TestProjectLoader temp;
    bool loaded = temp.Load(FilePath, FileName);
    Sequence::setFrames(temp.getFrames());
    Sequence::setName(FileName);
    //temp.ProjectLoader::~ProjectLoader();

    return loaded;
  }

  bool TestSequence::Load(map <string, float> &params, string FilePath, string FileName)
  {
    bool loaded = TestSequence::Load(FilePath, FileName);
    estimateUniformScale(params);
    computeInterpolation(params);

    return loaded;
  }

  TestSequence::TestSequence(string FilePath, string FileName)
  {
    TestSequence::Load(FilePath, FileName);
  }

  TestSequence::TestSequence(map <string, float> &params, string FilePath, string FileName)
  {
    TestSequence::Load(params, FilePath, FileName);
  }
//---------------------------------------------------------------------------------------
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

  void CompareSolves(vector<Solvlet> Solves, vector<Frame*> Frames, float AcceptableLinearError)
  {
    std::cout << "AcceptableLinearError = " << AcceptableLinearError << " pixels" << std::endl;
    ASSERT_GE(Solves.size(), 0);

    map<int,float> framesErrors;
    vector<float> partsErrors;
    bool tooLargeError = false;
    for (unsigned int i = 0; i < Frames.size(); i++)
      for (unsigned int s = 0; s < Solves.size(); s++)
      {
        if(Frames[i]->getID() == Solves[s].getFrameID())
        {
          Skeleton skeleton = Frames[i]->getSkeleton();
          map<int, pair<Point2f, Point2f>> PartLocations = getPartLocations(skeleton);
          vector<LimbLabel> Labels = Solves[s].getLabels();
          ASSERT_EQ(PartLocations.size(), Labels.size());
          float maxError = 0.0f;
          for (unsigned int k = 0; k < PartLocations.size(); k++)
          {
            //Calculate error
            std::pair<Point2f, Point2f> labelJointsLocation = Labels[k].getEndpoints(); // Actual current part joints locations      
            int partID = Labels[k].getLimbID();
            float error = getLinearError(PartLocations[partID], Labels[k]);
            partsErrors.push_back(error);
            if (error > maxError)
              maxError = error;

            //Compare            
            if (error > AcceptableLinearError)
            {
              tooLargeError = true;
              std::cout << "LockframeID = " << Frames[i]->getID() << ", PartID = " << partID << ", Error = " << round(error) << "(pixels)" << std::endl;
            }
            
          }
        framesErrors.emplace(Frames[i]->getID(), maxError);
      }
    }

    //cout << "partsErrors = " << partsErrors << endl;
    int framesSolved = 0, partsFound = 0;
    for (unsigned int i = 0; i < partsErrors.size(); i++)
      if (partsErrors[i] <= AcceptableLinearError)
        partsFound++;
    for(auto error = framesErrors.begin(); error != framesErrors.end(); ++error)
      if(error->second <= AcceptableLinearError)
        framesSolved++;

    std::cout << (static_cast<float>(partsFound) / static_cast<float>(partsErrors.size())) << "% of the body parts was found\n";
    std::cout << static_cast<float>(framesSolved) / static_cast<float>(framesErrors.size()) << "% of the sequence was solved correctly\n";
    EXPECT_FALSE(tooLargeError);
  }

   //Copyed from: http://www.juergenwiki.de/work/wiki/doku.php?id=public:hog_descriptor_computation_and_visualization
  vector<vector<vector<float>>> averageGradientStrengths(const cv::Mat & Image, std::vector<float>& descriptors, const cv::Size & winSize, const cv::Size & blockSize, const cv::Size & blockStride, const cv::Size & cellSize, const int nBins, const int derivAper, const double winSigma, const int histogramNormType, const double L2HysThresh, const bool gammaCorrection, const int nlevels)
  {
    //prepare data structure
    int cells_in_x_dir = winSize.width / cellSize.width;
    int cells_in_y_dir = winSize.height / cellSize.height;
    int totalnrofcells = cells_in_x_dir * cells_in_y_dir;
    float*** gradientStrengths = new float**[cells_in_y_dir];
    int** cellUpdateCounter = new int*[cells_in_y_dir];
    for (int y = 0; y<cells_in_y_dir; y++)
    {
      gradientStrengths[y] = new float*[cells_in_x_dir];
      cellUpdateCounter[y] = new int[cells_in_x_dir];
      for (int x = 0; x<cells_in_x_dir; x++)
      {
        gradientStrengths[y][x] = new float[nBins];
        cellUpdateCounter[y][x] = 0;
        for (int bin = 0; bin<nBins; bin++)
          gradientStrengths[y][x][bin] = 0.0;
      }
    }

    // compute gradient strengths per cell
    int blocks_in_x_dir = cells_in_x_dir - 1;
    int blocks_in_y_dir = cells_in_y_dir - 1;
    int descriptorDataIdx = 0;
    int cellx = 0;
    int celly = 0;

    for (int blockx = 0; blockx<blocks_in_x_dir; blockx++)
      for (int blocky = 0; blocky<blocks_in_y_dir; blocky++)
        for (int cellNr = 0; cellNr<4; cellNr++)
        {
          int cellx = blockx;
          int celly = blocky;
          if (cellNr == 1) celly++;
          if (cellNr == 2) cellx++;
          if (cellNr == 3)
          {
            cellx++;
            celly++;
          }
          for (int bin = 0; bin<nBins; bin++)
          {
            float gradientStrength = descriptors[descriptorDataIdx];
            descriptorDataIdx++;
            gradientStrengths[celly][cellx][bin] += gradientStrength;
          } 
          cellUpdateCounter[celly][cellx]++;
        }

    // prepare data structure
    vector<vector<vector<float>>> averageGradients(cells_in_y_dir);
    for (int i = 0; i < cells_in_y_dir; i++)
    {
      averageGradients[i].resize(cells_in_x_dir);
      for (int k = 0; k < cells_in_x_dir; k++)
        averageGradients[i][k].resize(nBins);
    }
    // compute average gradient strengths
    for (int celly = 0; celly<cells_in_y_dir; celly++)
      for (int cellx = 0; cellx<cells_in_x_dir; cellx++)
      {
        float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];
        for (int bin = 0; bin<nBins; bin++)
          averageGradients[celly][cellx][bin] = gradientStrengths[celly][cellx][bin] / NrUpdatesForThisCell;
      }

    // Clear
    for (int y = 0; y<cells_in_y_dir; y++)
    {
      for (int x = 0; x<cells_in_x_dir; x++)
        delete[] gradientStrengths[y][x];
      delete[] gradientStrengths[y];
      delete[] cellUpdateCounter[y];
    }
    delete[] gradientStrengths;
    delete[] cellUpdateCounter;

    return averageGradients;
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
void TestISM::computeISMcell(Frame* left, Frame* right, const int maxFrameHeight){}

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

 ostream& operator<<(ostream& fout, vector<float> x)
 {
   if (x.size() >0)
   {
     fout << "{" << "0:" << x[0];
     for (int i = 1; i < x.size(); i++)
       fout << ", " << i << ":" << x[i];
     fout << "}";
   }

   return fout;
 }

 void PutFormattedLabel(ostream &fout, LimbLabel label, string FieldsString, int LabelID, float linearError, float angleError)
 {
   if (FieldsString == "")
     FieldsString = {"PartID LabelID Angle Joints AvgScore "};

   vector<string> FieldsSet = { "PartID", "LabelID", "Angle", "Joints", "Center", "Polygon", "AvgScore", "Scores", "linearError", "angleError", ""};
   map<string, int> Fields;
   for (int i = 0; i < FieldsSet.size(); i++)
     Fields.emplace(pair<string, int>(FieldsSet[i], i));

   vector<int> FieldNum;
   string temp = "";
   for(int i = 0; i < FieldsString.size(); i++)
   {
     if(FieldsString[i] == ',' || FieldsString[i] == ' ' || FieldsString[i] == ';')
     {
       if(Fields.find(temp) != Fields.end())
         FieldNum.push_back(Fields.at(temp));
       temp.clear();
     }
     else
       temp = temp + FieldsString[i];
   }
   if(temp != "")
     FieldNum.push_back(Fields.at(temp));

   Point2f p0, p1;
   vector<Score> scores;
   vector<Point2f> P;
   label.getEndpoints(p0, p1);
   p0 = static_cast<Point2i>(p0);
   p1 = static_cast<Point2i>(p1);
   scores = label.getScores();
   P = label.getPolygon();

   fout << "  ";
   for(int i = 0; i < FieldNum.size(); i++)
   {
     int t = FieldNum[i];
     if (FieldNum[i] != 10) 
       fout << FieldsSet[t] << " = ";
     switch (t)
     {
       case 0: fout << label.getLimbID(); break;
       case 1: fout << LabelID; break;
       case 2: fout << label.getAngle(); break;
       case 3: fout << "{" << p0 << ", " << p1 << "}"; break;
       case 4: fout << label.getCenter(); break;
       case 5: fout << "{" << P[0] << ", " << P[1] << ", " << P[2] << ", " << P[3] << "}"; break;
       case 6: fout << label.getAvgScore(); break;
       case 7: 
               fout << "{";
               for(int k = 0; k < scores.size(); k++)
                 fout << scores[k].getScore() << ", ";
               fout << "}";
               break;
       case 8: fout << linearError; break;
       case 9: fout << angleError; break;
       case 10: break;
       
     }
     if (i != FieldNum.size() - 1)
     {
       if (FieldNum[i] != 10)
         fout << ", ";
     }
     else 
       fout << endl;
   }

   FieldsSet.clear();
   Fields.clear();
   FieldNum.clear();
   temp.clear();
   scores.clear();
   P.clear();
 }

 void PutLabels(ostream &fout, string fields, map<uint32_t, vector<LimbLabel>> &Labels, map<int, vector<float>> &LinearErrors, map<int, vector<float>> &AngleErrors, int TopLabelsCount)
 {
   if(TopLabelsCount != 0)
   {
     if(fields == "") fields = "PartID, Angle, Joints, Scores, linearError";
     fout << "\n=======================================================\n\n";
     fout <<"Top labels, grouped by part id:\n\n";
     for (unsigned int i = 0; i < Labels.size(); i++)
     {
       for (unsigned int k = 0; (k < Labels[i].size()) && (k < TopLabelsCount); k++)
         PutFormattedLabel(fout, Labels[i][k], fields, k, LinearErrors[i][k]);
       fout << endl;
     }
   }
   else
   {
     if (fields == "") fields = "PartID, LabelID, Angle, angleError, AvgScore, linearError";
     for (unsigned int i = 0; i < Labels.size(); i++)
     {
       fout << "\n=======================================================\n\n";
       fout << "All labels for BodyPart(" << i << "):\n\n";
       for (unsigned int k = 0; k < Labels[i].size(); k++)
         PutFormattedLabel(fout, Labels[i][k], fields, k, LinearErrors[i][k], AngleErrors[i][k]);
     }
   }

 }

 void PutSigificantErrors(ostream &fout, map<int, vector<float>>LinearErrors, map<int, vector<float>>AngleErrors, int TopLabelsCount)
 {
   vector<float> significantLinearErrors = MinLabelsError(LinearErrors, TopLabelsCount);
   vector<float> significantAngleErrors = MinLabelsError(AngleErrors, TopLabelsCount);
   float significantLinearError = *std::max_element(significantLinearErrors.begin(), significantLinearErrors.end());
   float significantAngleError = *std::max_element(significantAngleErrors.begin(), significantAngleErrors.end());
   fout << "Parts significant linear errors sorted by PartID, (pixels): " << significantLinearErrors << endl;
   fout << "Parts significant angle errors sorted by PartID, (degrees): " << significantAngleErrors << endl;
   fout << "The maximum linear error, (pixels): " << significantLinearError << endl;
   fout << "The maximum angle error, (degrees): " << significantAngleError << endl;   
 }

 // Return distance between ideal body part and label
 float getLinearError(pair<Point2f, Point2f> PartJoints, LimbLabel label, bool considerAngle)
 {
   Point2f l0, l1, p0, p1, delta0, delta1;
   p0 = PartJoints.first; // Ideal boby part point
   p1 = PartJoints.second; // Ideal boby part point
   label.getEndpoints(l0, l1); // Label points
   delta0 = l0 - p0;
   delta1 = l1 - p1;
   float error = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
   if (!considerAngle)
   {
     delta0 = l0 - p1;
     delta1 = l1 - p0;
     float mirroredPart_error = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
     error = min(error, mirroredPart_error);
   }

   return error;
 }

 vector<float> MinLabelsError(map<int, vector<float>> Errors, int TopLabelsCount)
 {
   vector<float> partsErrors; 
   for (int i = 0; i < Errors.size(); i++)
   {
     float error = INFINITY;
     for(int k = 0; k < Errors[i].size() && k < TopLabelsCount; k++)
       if(abs(Errors[i][k]) < error)
         error = abs(Errors[i][k]);
     partsErrors.push_back(error);
   }

   return partsErrors;
 }

 float getAngleError(pair<Point2f, Point2f> PartJoints, LimbLabel label)
 {
   float partAngle = spelHelper::getAngle(PartJoints.first, PartJoints.second);
   float labelAngle = label.getAngle();
   float error = partAngle - labelAngle;

   return error;
 }

 map<int, vector<float>> LabelsLinearErrors(map<uint32_t, vector<LimbLabel>> &Labels, Skeleton &pattern)
 {
   map<int, pair<Point2f, Point2f>> PartLocation = getPartLocations(pattern);
   map<int, vector<float>> Errors;
   for (unsigned int id = 0; id < Labels.size(); id++)
   {
     vector<float> partErrors;
     for (int k = 0; k < static_cast<int>(Labels[id].size()); k++)
     {
       float linearError = getLinearError(PartLocation[id], Labels[id][k]); // Distance between ideal body part and label
       partErrors.push_back(linearError);
     }
     Errors.emplace(pair<int, vector<float>>(id, partErrors));
   }

   return Errors;
 }

 map<int, vector<float>> LabelsAngleErrors(map<uint32_t, vector<LimbLabel>> &Labels, Skeleton &pattern)
 {
   map<int, pair<Point2f, Point2f>> PartLocation = getPartLocations(pattern);
   map<int, vector<float>> Errors;
   for (unsigned int id = 0; id < Labels.size(); id++)
   {
     vector<float> partErrors;
     for (int k = 0; k < static_cast<int>(Labels[id].size()); k++)
     {
       float angleError = getAngleError(PartLocation[id], Labels[id][k]); // Distance between ideal body part and label
       partErrors.push_back(angleError);
     }
     Errors.emplace(pair<int, vector<float>>(id, partErrors));
   }

   return Errors;
 }

 map<int, map<int,LimbLabel>> selectEffectiveLabels(map<uint32_t, vector<LimbLabel>> &Labels, map<int, vector<Point2f>> &Errors, float TolerableLinearError)
 {
   map<int, map<int, LimbLabel>> effectiveLabels;
   for (unsigned int id = 0; id < Labels.size(); id++)
   {
     map<int, LimbLabel> partEffectiveLabels;
     for (int k = 0; k < static_cast<int>(Labels[id].size()); k++)
     {
       if (Errors[id][k].x <= TolerableLinearError && Labels[id][k].getAvgScore() >= 0)
         partEffectiveLabels.emplace(pair<int, LimbLabel>(k, Labels[id][k]));
     }
     effectiveLabels.emplace(pair<int, map<int, LimbLabel>>(id, partEffectiveLabels));
   }

   return effectiveLabels;
 }

 vector<int> selectNotFoundedParts(Skeleton &skeleton, map<int, vector<float>> &Errors, map<uint32_t,vector<LimbLabel>> &Labels, int TolerableError, int TopLabelsCount)
 {
   vector<int> notFoundedParts;
   //int n = (TopLabelsCount == 0) ? Errors.size() : TopLabelsCount;
   for (int i = 0; i < Errors.size(); i++)
   {
     bool partFound = false;
     for (int k = 0; k < TopLabelsCount && k < Errors[i].size(); k++)
       if (Errors[i][k] < TolerableError && Labels[i][k].getAvgScore() >=  0.0f)
         partFound = true;
     BodyPart *bodyPart = skeleton.getBodyPart(i);
     if (!partFound && !bodyPart->getIsOccluded())
       notFoundedParts.push_back(i);
   }

   return notFoundedParts;
 }

}