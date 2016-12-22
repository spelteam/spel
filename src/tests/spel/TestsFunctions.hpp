#include "spelHelper.hpp"
#include "projectLoader.hpp"
#include "colorHistDetector.hpp"
#include "frame.hpp"
#include "imagesimilaritymatrix.hpp"
#include <sequence.hpp>
#include <fstream>
#include <iostream>
  
namespace SPEL
{ 
  const int NBins = 8;
  
  const uint8_t Factor = static_cast<uint8_t> (ceil(pow(2, 8) / NBins)); // Colorspace scaling coefficient Model.nBins

  void PutPartRect(Mat &Image, vector<Point2f> polygon, Scalar color); // Drawing the part polygon
  map<int, spelRECT<Point2f>> SkeletonRects(Skeleton skeleton); // Build the rectangles for all of bodyparts
  spelRECT<Point2f> BuildPartRect(BodyJoint *j0, BodyJoint *j1, float LWRatio); // Build rectangle on bodypart joints
  map<int, pair<Point2f, Point2f>> getPartLocations(Skeleton skeleton); // Selecting locations of all body part from skeleton																		
  bool IsCrossed(vector<Point2f> PolygonPoints, spelRECT<Point2f> rect); // Returns "true" if polygon is crossed (occluded) by "rect"
  bool IsCrossed(spelRECT<Point2f> rect1, spelRECT<Point2f> rect2); // Returns "true" if "rect1" is crossed (occluded) by "rect2"
  vector<vector<pair<int, int>>> CrossingsList(map<int, spelRECT<Point2f>> Rects, map<int, int> depth); // For the each polygon select all polygons, which crossed it 
  vector <Point3i> GetPartColors(Mat image, Mat mask, spelRECT < Point2f > rect); // Build set of the rect pixels colours 
  void  Normalize(vector <vector <vector <float>>> &Histogramm, int nBins, int pixelsCount); // Normalization of the  histogram
  void PutHistogram(ofstream &fout, vector <vector <vector <float>>> &Histogramm, int sizeFG); // Output histogram into text file
 
  ostream& operator<<(ostream& fout, vector<float> x);

  // Write formatted limbLabel into text file. FieldsString - set of fields, that will be writed
  // For writing all fields 'FieldsString' must bee: "PartID, LabelID, Angle, Joints, Center, Polygon, AvgScore, Scores, linearError, angleError" 
  void PutFormattedLabel(ostream &fout, LimbLabel label, string FieldsString = "", int LabelID = 0, float linearError = INFINITY, float angleError = INFINITY);

  // Put all labels
  void PutLabels(ostream &fout, string fields, map<uint32_t, vector<LimbLabel>> &Labels, map<int, vector<float>> &LinearErrors, map<int, vector<float>> &AngleErrors, int TopLabelsCount);

  //Put Detect Results
  void PutSigificantErrors(ostream &fout, map<int, vector<float>>LinearErrors, map<int, vector<float>>AngleErrors, int TopLabelsCount);

  // Calculation maximum distance between the label and real body part
  float getLinearError(pair<Point2f, Point2f> PartJoints, LimbLabel label, bool considerAngle = true);
  float getAngleError(pair<Point2f, Point2f> PartJoints, LimbLabel label);

  // Calculation distances from all labels to real skeleton
  // map<int, vector<Point2f>> getLabelsErrors(map<uint32_t, vector<LimbLabel>> &Labels, Skeleton &pattern); // Point2f(linearError, angleError)
  map<int, vector<float>> LabelsLinearErrors(map<uint32_t, vector<LimbLabel>> &Labels, Skeleton &pattern);
  map<int, vector<float>> LabelsAngleErrors(map<uint32_t, vector<LimbLabel>> &Labels, Skeleton &pattern);
  map<int, map<int, LimbLabel>> selectEffectiveLabels(map<uint32_t, vector<LimbLabel>> &Labels, map<int, vector<Point2f>> &Errors, float TolerableLinearError);
  vector<float> MinLabelsError(map<int, vector<float>> Errors, int TopLabelsCount);
  long clock_to_ms(long ExecTime);

  vector<int> selectNotFoundedParts(Skeleton &skeleton, map<int, vector<float>> &Errors, map<uint32_t, vector<LimbLabel>> &Labels, int TolerableLinearError, int TopLabelsCount = 0);

  class TestProjectLoader : public ProjectLoader
  {
  public:
    TestProjectLoader();
    TestProjectLoader(string FilePath, string FileName);
    bool Load(string FilePath, string FileName);
  };
  class TestSequence : public Sequence
  {
  public:
    bool Load(string FilePath, string FileName);
    bool Load(map <string, float> &params, string FilePath, string FileName);
    TestSequence(string FilePath, string FileName);
    TestSequence(map <string, float> &params, string FilePath, string FileName);
  };

  int keyFramesCount(vector<Frame*> frames); // Counting of keyframes in set of frames 
  int FirstKeyFrameNum(vector<Frame*> frames); // Returns  index of first keyframe

  vector<Point2f> getPartRect(float LWRatio, Point2f p0, Point2f p1); // building a part rectangle on part joints, == DetectorGetPartRect, but used another way
  vector<Point2f> getPartRect(float LWRatio, Point2f p0, Point2f p1, cv::Size blockSize);

  void CompareSolves(vector<Solvlet> Solves, vector<Frame*> Frames, ImageSimilarityMatrix &ISM);
  vector<vector<vector<float>>> averageGradientStrengths(const cv::Mat & Image, std::vector<float>& descriptors, const cv::Size & winSize, const cv::Size & blockSize, const cv::Size & blockStride, const cv::Size & cellSize, const int nBins, const int derivAper, const double winSigma, const int histogramNormType, const double L2HysThresh, const bool gammaCorrection, const int nlevels);
  
  class TestISM : public ImageSimilarityMatrix
  {
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
  private:
    virtual void computeISMcell(Frame* left, Frame* right, const int maxFrameHeight);
  };

}