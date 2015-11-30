#include "spelHelper.hpp"
#include "projectLoader.hpp"
#include "colorHistDetector.hpp"
#include "frame.hpp"
#include "imagesimilaritymatrix.hpp"
#include <fstream>
#include <iostream>
  
namespace SPEL
{ 
  const int NBins = 8;
  
  const uint8_t Factor = static_cast<uint8_t> (ceil(pow(2, 8) / NBins)); // Colorspace scaling coefficient Model.nBins

  void PutPartRect(Mat &Image, vector<Point2f> polygon, Scalar color); // Drawing the part polygon
  map<int, POSERECT<Point2f>> SkeletonRects(Skeleton skeleton); // Build the rectangles for all of bodyparts
  POSERECT<Point2f> BuildPartRect(BodyJoint *j0, BodyJoint *j1, float LWRatio); // Build rectangle on bodypart joints
  map<int, pair<Point2f, Point2f>> getPartLocations(Skeleton skeleton); // Selecting locations of all body part from skeleton
  bool IsCrossed(vector<Point2f> PolygonPoints, POSERECT<Point2f> rect); // Returns "true" if polygon is crossed (occluded) by "rect"
  bool IsCrossed(POSERECT<Point2f> rect1, POSERECT<Point2f> rect2); // Returns "true" if "rect1" is crossed (occluded) by "rect2"
  vector<vector<pair<int, int>>> CrossingsList(map<int, POSERECT<Point2f>> Rects, map<int, int> depth); // For the each polygon select all polygons, which crossed it 
  vector <Point3i> GetPartColors(Mat image, Mat mask, POSERECT < Point2f > rect); // Build set of the rect pixels colours 
  void  Normalize(vector <vector <vector <float>>> &Histogramm, int nBins, int pixelsCount); // Normalization of the  histogram
  void PutHistogram(ofstream &fout, vector <vector <vector <float>>> &Histogramm, int sizeFG); // Output histogram into text file
  vector<Frame*> LoadTestProject(string FilePath, string FileName); // Loading frames from project
  //map <string, float> SetParams(vector<Frame*> frames, Sequence **seq); // Set parameters from the frames sequence
  vector<Frame*> LoadTestProject(map <string, float> &params, string FilePath, string FileName); 

  int keyFramesCount(vector<Frame*> frames); // Counting of keyframes in set of frames 
  int FirstKeyFrameNum(vector<Frame*> frames); // Returns  index of first keyframe

  vector<Point2f> getPartRect(float LWRatio, Point2f p0, Point2f p1); // building a part rectangle on part joints, == DetectorGetPartRect, but used another way
  vector<Point2f> getPartRect(float LWRatio, Point2f p0, Point2f p1, cv::Size blockSize);

  void CompareSolves(vector<Solvlet> Solves, vector<Frame*> Frames, ImageSimilarityMatrix &ISM);

  class TestISM : public ImageSimilarityMatrix
  {
  private:
    virtual void computeISMcell(const Frame* left, const Frame* right, const int maxFrameHeight);
  public:	
    TestISM(void);
    TestISM(const TestISM &m);
    TestISM(const std::vector<Frame*> &frames);
    TestISM(TestISM &&m);
    virtual ~TestISM(void);
    virtual TestISM & operator=(const TestISM &s);
    // Building normalized ISM of the frames sequence
    // "useOverlapFactor" chooses gradual ("true") or abrupt("false") icrease assesment of the mask mismatch
    void build(vector<Frame*> frames, bool useOverlapFactor); 
    };

  


}