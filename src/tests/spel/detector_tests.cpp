// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif
#include <gtest/gtest.h>
#include <detector.hpp>
#include <colorHistDetector.hpp>
#include <surfDetector.hpp>
#include <lockframe.hpp>
#include <fstream>
#include "TestsFunctions.hpp"

using namespace cv;
using namespace std;

namespace SPEL
{
  class TestingDetector : public ColorHistDetector
  {
  public:
    Mat DeRotate(Mat imgSource, spelRECT <Point2f> &initialRect, float angle, Size size);
    float GetBoneLength(Point2f begin, Point2f end);
    float GetBoneWidth(float length, BodyPart bodyPart);
  };

  Mat TestingDetector::DeRotate(Mat imgSource, spelRECT <Point2f> &initialRect, float angle, Size size)
  {
    return  spelHelper::rotateImageToDefault(imgSource, initialRect, angle, size);
  }

  float TestingDetector::GetBoneLength(Point2f begin, Point2f end)
  {
    return BodyPart::getBoneLength(begin, end);
  }

  float TestingDetector::GetBoneWidth(float length, BodyPart bodyPart)
  {
    return  BodyPart::getBoneWidth(length, bodyPart);
  }

  spelRECT<Point2f> CreateRect(float x1, float x2, float y1, float y2)
  {
    Point2f a(x1, y1), b(x2, y1), c(x2, y2), d(x1, y2), E(0, 0);
    spelRECT <Point2f> rect(a, b, c, d);
    return rect;
  }

  // Rotation of the rectangle around center 
  spelRECT<Point2f> RotateRect(spelRECT<Point2f> &rect, float angle)
  {
    spelRECT<Point2f> RotatedRect;
    Point2f center = rect.GetCenter<Point2f>();
    RotatedRect.point1 = spelHelper::rotatePoint2D(rect.point1, center, angle);
    RotatedRect.point2 = spelHelper::rotatePoint2D(rect.point2, center, angle);
    RotatedRect.point3 = spelHelper::rotatePoint2D(rect.point3, center, angle);
    RotatedRect.point4 = spelHelper::rotatePoint2D(rect.point4, center, angle);
    return RotatedRect;
  }

  // Filling the rectangle
  void FillRect(Mat &Img, spelRECT<Point2f> &rect, Vec3b colour)
  {
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        Img.at<Vec3b>(j, i) = colour;
  }

  void FillRotatedRect(Mat &Img, spelRECT<Point2f> &rect, Vec3b colour)
  {
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (rect.containsPoint(Point2f((float)i, (float)j)) == 1)
          Img.at<Vec3b>(j, i) = colour;
  }

  void FillRectRand(Mat &Img, spelRECT<Point2f> &rect)
  {
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Vec3b colour;
    const int c = 250;
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
      {
        colour = Vec3b(uchar(rand() * c / RAND_MAX), uchar(rand() * c / RAND_MAX), uchar(rand() * c / RAND_MAX));
        Img.at<Vec3b>(j, i) = colour;
      }
  }

  // Rotation the rectangle image
  Mat RotateImage(Mat Img, spelRECT<Point2f> &rect, float angle)
  {
    Size size = Img.size();
    Mat img2 = Mat(size, CV_8UC3, Scalar(0, 0, 0));
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Point2f center = rect.GetCenter<Point2f >();
    Point2f E;
    for (int i = (int)xmin; i <= (int)xmax; i++)
    {
      for (int j = (int)ymin; j <= (int)ymax; j++)
      {
        E = Point2i(i, j);
        E = spelHelper::rotatePoint2D(E, center, angle);
        if ((0 < E.x) && (E.x < size.width) && (0 < E.y) && (E.y < size.height))
          img2.at<Vec3b>((int32_t)(E.y), (int32_t)(E.x)) = Img.at<Vec3b>(j, i);
      }
    }
    return img2;
  }

  // Searching the rectangle extreme coordinates at image
  void GetExtremePoints(Mat Img, Vec3b bgColour, Point2f &Min, Point2f &Max)
  {
    Size size = Img.size();
    Min = Point2f((float)size.width + 1, (float)size.height + 1);
    Max = Point2f(-1, -1);
    for (int i = 0; i < size.width; i++)
      for (int j = 0; j < size.height; j++)
        if (Img.at<Vec3b>(j, i) != bgColour)
        {
          if (i < (int)Min.x) { Min.x = (float)i; }
          if (i > (int)Max.x) { Max.x = (float)i; }
          if (j < (int)Min.y) { Min.y = (float)j; }
          if (j > (int)Max.y) { Max.y = (float)j; }
        }
  }

  TEST(DetectorTests, getID_SetID)
  {
    int id = 18;
    Detector* d = new ColorHistDetector(8);
    d->setID(id);
    EXPECT_EQ(id, d->getID());

    delete d;
    d = new HogDetector();
    d->setID(id);
    EXPECT_EQ(id, d->getID());

    delete d;
    d = new SurfDetector();
    d->setID(id);
    EXPECT_EQ(id, d->getID());
  }


  TEST(DetectorTests, rotateImageToDefault_OneColor)
  {
    // Prepare input data
    int rows = 200, cols = 200; // image size
    float angle = 45; // the rotation angle
    float x1 = 10.0, x2 = 40.0, y1 = 20.0, y2 = 60.0; // the rectangle vertices
    spelRECT <Point2f> rect = CreateRect(x1, x2, y1, y2);
    spelRECT <Point2f> RotatedRect = RotateRect(rect, angle);
    Point2f center = rect.GetCenter<Point2f >();
    Point2f center2 = RotatedRect.GetCenter<Point2f >();

    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Point2f MinR(xmin, ymin);
    Point2f MaxR(xmax, ymax);

    Mat img1 = Mat(Size(cols, rows), CV_8UC3, Scalar(0, 0, 0));
    Mat img2 = Mat(Size(cols, rows), CV_8UC3, Scalar(0, 0, 0));

    Vec3b colour(0, 0, 255); // color fill 

    FillRect(img1, rect, colour);
    FillRotatedRect(img2, RotatedRect, colour);

    TestingDetector chd;
    Size size((int)(xmax - xmin + 1), (int)(ymax - ymin + 1));

    // Testing
    Mat  X = chd.DeRotate(img2, RotatedRect, angle, size);

    imwrite("image.jpg", img1);
    imwrite("rotated_image.jpg", img2);
    imwrite("derotated_image.jpg", X);

    Point2f Min, Max;

    // Check X image
    GetExtremePoints(X, Vec3b(255, 255, 255), Min, Max);

    // Checking extreme points
    float delta = 1; // tolerable linear error
    EXPECT_LE(Min.x, delta);
    EXPECT_LE(abs(Max.x + xmin - MaxR.x), delta);
    EXPECT_LE(Min.y, delta);
    EXPECT_LE(abs(Max.y + ymin - MaxR.y), delta);

    // Checking matching of the images points with one fill color 
    uint32_t S = 0;
    uint32_t S0 = (uint32_t)((xmax - xmin + 1)*(ymax - ymin + 1));
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img1.at<Vec3b>(j, i) == X.at<Vec3b>((int)(j - ymin), (int)(i - xmin)))
          S++;


    float epsilon = 20; // tolerable error of the matching points number, %
    float FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    img1.release();
    img2.release();
    X.release();
  }

  TEST(DetectorTests, rotateImageToDefault_RandColor)
  {
    int rows = 200, cols = 200; // image size
    float angle = 45; // the rotetion angle
    float x1 = 10.0, x2 = 40.0, y1 = 20.0, y2 = 60.0; // the rectangle vertices
    spelRECT <Point2f> rect = CreateRect(x1, x2, y1, y2);
    spelRECT <Point2f> RotatedRect = RotateRect(rect, angle);
    Point2f center = rect.GetCenter<Point2f >();
    Point2f center2 = RotatedRect.GetCenter<Point2f >();

    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Point2f MinR(xmin, ymin);
    Point2f MaxR(xmax, ymax);

    Mat img1 = Mat(Size(cols, rows), CV_8UC3, Scalar(0, 0, 0));
    Mat img2 = Mat(Size(cols, rows), CV_8UC3, Scalar(0, 0, 0));

    FillRectRand(img1, rect);
    img2 = RotateImage(img1, rect, angle);

    TestingDetector chd;
    Size size((int)(xmax - xmin + 1), (int)(ymax - ymin + 1));

    spelRECT <Point2f> RotatedRectCopy = RotatedRect;

    Mat  X = chd.DeRotate(img2, RotatedRect, angle, size);

    imwrite("image_RandColor.jpg", img1);
    imwrite("rotated_image_RandColor.jpg", img2);
    imwrite("derotated_image_RandColor.jpg", X);

    Point2f Min, Max;

    // Check X image
    GetExtremePoints(X, Vec3b(255, 255, 255), Min, Max);

    // Checking extreme points
    float delta = 1; // tolerable linear error
    EXPECT_LE(Min.x, delta);
    EXPECT_LE(abs(Max.x + xmin - MaxR.x), delta);
    EXPECT_LE(Min.y, delta);
    EXPECT_LE(abs(Max.y + ymin - MaxR.y), delta);

    // Checking matching of the images points with one fill color 
    uint32_t S = 0;
    uint32_t S0 = (uint32_t)((xmax - xmin + 1)*(ymax - ymin + 1));
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img1.at<Vec3b>(j, i) == X.at<Vec3b>(((int)(j - ymin), (int)(i - xmin))))
          S++;

    float epsilon = 100; // tolerable error of the matching points number, %
    float FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    img1.release();
    img2.release();
    X.release();
  }

  TEST(DetectorTests, rotateImageToDefault_FileImage)
  {
#ifdef WINDOWS
#ifdef DEBUG
    Mat img1, img2, img3, img4, Q1, Q2, Q3, Q4;
    if (IsDebuggerPresent())
    {
      img1 = imread("Debug/speltests_TestData/ImageRotationTestData/image1.jpg");
      img2 = imread("Debug/speltests_TestData/ImageRotationTestData/image2.jpg");
      img3 = imread("Debug/speltests_TestData/ImageRotationTestData/image3.jpg");
      img4 = imread("Debug/speltests_TestData/ImageRotationTestData/image4.jpg");
      Q1 = imread("Debug/speltests_TestData/ImageRotationTestData/Q1.jpg");
      Q2 = imread("Debug/speltests_TestData/ImageRotationTestData/Q2.jpg");
      Q3 = imread("Debug/speltests_TestData/ImageRotationTestData/Q3.jpg");
      Q4 = imread("Debug/speltests_TestData/ImageRotationTestData/Q4.jpg");
    }
    else
    {
      img1 = imread("speltests_TestData/ImageRotationTestData/image1.jpg");
      img2 = imread("speltests_TestData/ImageRotationTestData/image2.jpg");
      img3 = imread("speltests_TestData/ImageRotationTestData/image3.jpg");
      img4 = imread("speltests_TestData/ImageRotationTestData/image4.jpg");
      Q1 = imread("speltests_TestData/ImageRotationTestData/Q1.jpg");
      Q2 = imread("speltests_TestData/ImageRotationTestData/Q2.jpg");
      Q3 = imread("speltests_TestData/ImageRotationTestData/Q3.jpg");
      Q4 = imread("speltests_TestData/ImageRotationTestData/Q4.jpg");
    }
#else
    Mat img1 = imread("Release/speltests_TestData/ImageRotationTestData/image1.jpg");
    Mat img2 = imread("Release/speltests_TestData/ImageRotationTestData/image2.jpg");
    Mat img3 = imread("Release/speltests_TestData/ImageRotationTestData/image3.jpg");
    Mat img4 = imread("Release/speltests_TestData/ImageRotationTestData/image4.jpg");
    Mat Q1 = imread("Release/speltests_TestData/ImageRotationTestData/Q1.jpg");
    Mat Q2 = imread("Release/speltests_TestData/ImageRotationTestData/Q2.jpg");
    Mat Q3 = imread("Release/speltests_TestData/ImageRotationTestData/Q3.jpg");
    Mat Q4 = imread("Release/speltests_TestData/ImageRotationTestData/Q4.jpg");
#endif  // DEBUG
#else
    Mat img1 = imread("speltests_TestData/ImageRotationTestData/image1.jpg");
    Mat img2 = imread("speltests_TestData/ImageRotationTestData/image2.jpg");
    Mat img3 = imread("speltests_TestData/ImageRotationTestData/image3.jpg");
    Mat img4 = imread("speltests_TestData/ImageRotationTestData/image4.jpg");
    Mat Q1 = imread("speltests_TestData/ImageRotationTestData/Q1.jpg");
    Mat Q2 = imread("speltests_TestData/ImageRotationTestData/Q2.jpg");
    Mat Q3 = imread("speltests_TestData/ImageRotationTestData/Q3.jpg");
    Mat Q4 = imread("speltests_TestData/ImageRotationTestData/Q4.jpg");
#endif  // WINDOWS
    imwrite("Q01.jpg", Q1);
    imwrite("Q02.jpg", Q2);
    imwrite("Q03.jpg", Q3);
    imwrite("Q04.jpg", Q4);

    // Prepare input data
    Size img_size = img1.size();

    float angle = 45; // the rotetion angle
    float x1 = 10.0, x2 = 39.0, y1 = 20.0, y2 = 59.0; // the rectangle vertices
    spelRECT <Point2f> rect = CreateRect(x1, x2, y1, y2);
    spelRECT <Point2f> RotatedRect = RotateRect(rect, angle);
    Point2f center = rect.GetCenter<Point2f >();

    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Point2f MinR(xmin, ymin);
    Point2f MaxR(xmax, ymax);

    TestingDetector chd;
    Size size((int)(xmax - xmin + 1), (int)(ymax - ymin + 1));

    // Testing

    Mat  X1 = chd.DeRotate(Q1, RotatedRect, angle, size);
    Mat  X2 = chd.DeRotate(Q2, RotatedRect, angle, size);
    Mat  X3 = chd.DeRotate(Q3, RotatedRect, angle, size);
    Mat  X4 = chd.DeRotate(Q4, RotatedRect, angle, size);

    imwrite("derotated_image1.bmp", X1);
    imwrite("derotated_image2.bmp", X2);
    imwrite("derotated_image3.bmp", X3);
    imwrite("derotated_image4.bmp", X4);

    Point2f Min1, Max1;
    Point2f Min2, Max2;
    Point2f Min3, Max3;
    Point2f Min4, Max4;

    // Check X image
    GetExtremePoints(X1, Vec3b(255, 255, 255), Min1, Max1);
    GetExtremePoints(X2, Vec3b(255, 255, 255), Min2, Max2);
    GetExtremePoints(X3, Vec3b(255, 255, 255), Min3, Max3);
    GetExtremePoints(X4, Vec3b(255, 255, 255), Min4, Max4);

    // Checking extreme points
    float delta = 1; // tolerable linear error
    EXPECT_LE(Min1.x, delta);
    EXPECT_LE(abs(Max1.x + xmin - MaxR.x), delta);
    EXPECT_LE(Min1.y, delta);
    EXPECT_LE(abs(Max1.y + ymin - MaxR.y), delta);

    EXPECT_LE(Min2.x, delta);
    EXPECT_LE(abs(Max2.x + xmin - MaxR.x), delta);
    EXPECT_LE(Min2.y, delta);
    EXPECT_LE(abs(Max2.y + ymin - MaxR.y), delta);

    // Checking matching of the images points with one fill color 
    float epsilon = 75; // tolerable error of the matching points number, %

    uint32_t S = 0;
    uint32_t S0 = (uint32_t)((xmax - xmin + 1)*(ymax - ymin + 1));

    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img1.at<Vec3b>(j, i) == X1.at<Vec3b>((int)(j - ymin), (int)(i - xmin)))
          S++;
    float FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    S = 0;
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img2.at<Vec3b>(j, i) == X2.at<Vec3b>((int)(j - ymin), (int)(i - xmin)))
          S++;
    FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    S = 0;
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img3.at<Vec3b>(j, i) == X3.at<Vec3b>((int)(j - ymin), (int)(i - xmin)))
          S++;
    FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    S = 0;
    for (int i = (int)xmin; i <= (int)xmax; i++)
      for (int j = (int)ymin; j <= (int)ymax; j++)
        if (img4.at<Vec3b>(j, i) == X4.at<Vec3b>((int)(j - ymin), (int)(i - xmin)))
          S++;
    FalsePixels = (float)(100 * (S0 - S) / S0);
    cout << "FalsePixels:\t" << FalsePixels << "\tepsilon:\t" << epsilon << endl;
    EXPECT_LE(FalsePixels, epsilon);

    img1.release();
    img2.release();
    img3.release();
    img4.release();

    Q1.release();
    Q2.release();
    Q3.release();
    Q4.release();

    X1.release();
    X2.release();
    X3.release();
    X4.release();
  }

  TEST(DetectorTests, getFrame)
  {
    //Load the input data
    ColorHistDetector CHD;
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    CHD.m_frames = project.getFrames();

    //Create actual value
    Frame* frame = CHD.getFrame(0);

    //Compare
    EXPECT_EQ(CHD.m_frames[0]->getFrametype(), frame->getFrametype());
    EXPECT_EQ(CHD.m_frames[0]->getID(), frame->getID());
    EXPECT_EQ(CHD.m_frames[0]->getFrameSize(), frame->getFrameSize());
    EXPECT_EQ(CHD.m_frames[0]->getSkeleton(), frame->getSkeleton());
    EXPECT_EQ(CHD.m_frames[0]->getImageSize(), frame->getImageSize());
    EXPECT_EQ(CHD.m_frames[0]->getMaskSize(), frame->getMaskSize());

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
  }

  TEST(DetectorTests, getBoneLength)
  {
    TestingDetector detector;
    Point2f begin(0, 0), end(1, 0);

    float length = detector.GetBoneLength(begin, end);
    cout << length << endl;
    EXPECT_EQ(end.x - begin.x, length);
  }

  TEST(DetectorTests, getBoneWidth)
  {
    TestingDetector detector;
    BodyPart part;
    float lwRatio = 2.0;
    part.setLWRatio(lwRatio);
    float length = 1;

    float width = detector.GetBoneWidth(length, part);
    cout << length << endl;
    EXPECT_EQ(length / lwRatio, width);
  }

  //Output limbLabels set into text file
  void PutLimbLabels(ofstream &fout, string S, map <uint32_t, vector <LimbLabel>> X)
  {
    fout << S << endl;
    for (unsigned int p = 0; p < X.size(); p++)
    {
      for (unsigned int i = 0; i < X[p].size(); i++)
      {
        fout << endl << "  limbID " << X[p][i].getLimbID() << ", angle " << X[p][i].getAngle() << ", poligon" << X[p][i].getPolygon() << ", scores ";
        vector<Score> Scores = X[p][i].getScores();
        for (unsigned  int k = 0; k < Scores.size(); k++)
          fout << Scores[k].getScore() << ", ";
      }
      fout << endl;
    }
    fout << endl << "===========================================" << endl;
  }

  /*
  //Temporary test. Data loss is not checked - only repeating
  //The test for universal function "merge"
  TEST(DetectorTests, merge)
  {
  vector <vector <LimbLabel>> A;
  vector <vector <LimbLabel>> B;

  //Create polygons
  int polygonsCount = 10;
  vector<vector<Point2f>> Polygons;
  for (int i = 0; i < polygonsCount; i++)
  Polygons.push_back(vector < Point2f > { Point2f(i, i) });

  //Craeate set of limbLabels "A"
  int partsCount = 2;
  int A_size = 9;
  for (int p = 0; p < partsCount; p++)
  {
    vector<LimbLabel> temp_partLabels;
    for (int i = 0; i < A_size; i++)
    {
      int N = (Polygons.size() - 1)*rand() / RAND_MAX;
      LimbLabel temp_label(p, Point2f(N, N), N, Polygons[N], vector < Score > { Score(float((rand()) * 100 / RAND_MAX) / 100, "", 1)});
      temp_partLabels.push_back(temp_label);
    }
    A.push_back(temp_partLabels);
  }

  //Craeate set of limbLabels "B"
  int B_size = 9;
  for (int p = 0; p < partsCount; p++)
  {
    vector<LimbLabel> temp_partLabels;
    for (int i = 0; i < B_size; i++)
    {
      int N = (Polygons.size() - 1)*rand() / RAND_MAX;
      LimbLabel temp_label(p, Point2f(N, N), N, Polygons[N], vector < Score > { Score(float((rand()) * 100 / RAND_MAX) / 100, "", 1)});
      temp_partLabels.push_back(temp_label);
    }
    B.push_back(temp_partLabels);
  }

  //Run "merge"
  ColorHistDetector D;
  vector <vector <LimbLabel>> H = D.merge(A, B);
  vector <vector <LimbLabel>> A1 = D.merge(A, A);
  vector <vector <LimbLabel>> B1 = D.merge(B, B);
  vector <vector <LimbLabel>> C = D.merge(A1, B1);

  //Put all input and output labels sets into text file
  ofstream fout("Detector_merge.txt");
  PutLimbLabels(fout,"LimbLabels set A:", A);
  PutLimbLabels(fout,"LimbLabels set B:", B);
  PutLimbLabels(fout, "merge(A, B):", H);
  PutLimbLabels(fout, "merge(A, A):", A1);
  PutLimbLabels(fout,"merge(B, B):", B1);
  PutLimbLabels(fout, "merge(merge(A, A), merge(B, B)) :", C);
  fout.close();

  cout << "See input abd output limbLabels values in the file  'Detector_merge.txt'\n";

  //Compare
  for (int p = 0; p < C.size(); p++)
    for (int i = 0; i < C[p].size(); i++)
    {
      //Search equal polygon values in curent part labels
      for (int k = 0; k < C[p].size(); k++)
        if (i != k) EXPECT_FALSE(C[p][i].getPolygon() == C[p][k].getPolygon()) << "partID = " << p << ": polygons of label_Num " << i << " and " << k << "is equal" << endl;
      vector<Score> LimbScores = C[p][i].getScores();
      //Search equal scores in curent label
      for (int k = 0; k < LimbScores.size(); k++)
        for (int t = 0; t < LimbScores.size(); t++)
      if (t != k) EXPECT_FALSE(LimbScores[k] == LimbScores[t]) << ", PartID = " << p << ": Equal scores in label with Num = " << i << endl;
    }
  }
  */

  TEST(DetectorTests, merge)
  {
    map <uint32_t, vector <LimbLabel>> A;
    map <uint32_t, vector <LimbLabel>> B;

    //Create polygons
    int polygonsCount = 10;
    vector<vector<Point2f>> Polygons;
    for (int i = 0; i < polygonsCount; i++)
        Polygons.push_back(vector < Point2f > { Point2f(static_cast<float>(i), static_cast<float>(i)) });

    //Craeate sets of limbLabels "A" and ""B
    int partsCount = 2;
    for (int p = 0; p < partsCount; p++)
    {
      vector<LimbLabel> tempA_partLabels;
      vector<LimbLabel> tempB_partLabels;
      for (int N = 0; N < polygonsCount; N++)
      {
        float n = static_cast<float>(N);
        LimbLabel temp_label(p, Point2f(n, n), n, Polygons[N], vector < Score > { Score(n / (float)polygonsCount, "", 1.0f)});
        if (rand() > 0.3*RAND_MAX)
          tempA_partLabels.push_back(temp_label);
        if (rand() > 0.3*RAND_MAX)
          tempB_partLabels.push_back(temp_label);
      }
      A.insert(pair<uint32_t, vector <LimbLabel>>(p, tempA_partLabels));
      B.insert(pair<uint32_t, vector <LimbLabel>>(p, tempB_partLabels));
      tempA_partLabels.clear();
      tempB_partLabels.clear();
    }

    //Run "merge"
    ColorHistDetector D;
    auto H = D.merge(A, B, map<uint32_t, vector<LimbLabel>>());
    auto A1 = D.merge(A, A, map<uint32_t, vector<LimbLabel>>());
    auto B1 = D.merge(B, B, map<uint32_t, vector<LimbLabel>>());
    auto C = D.merge(A1, B1, map<uint32_t, vector<LimbLabel>>());

    //Put all input and output labels sets into text file
    ofstream fout("Detector_merge.txt");
    PutLimbLabels(fout, "LimbLabels set A:", A);
    PutLimbLabels(fout, "LimbLabels set B:", B);
    PutLimbLabels(fout, "merge(A, B):", H);
    PutLimbLabels(fout, "merge(A, A):", A1);
    PutLimbLabels(fout, "merge(B, B):", B1);
    PutLimbLabels(fout, "merge(merge(A, A), merge(B, B)) :", C);
    fout.close();

    cout << "See input abd output limbLabels values in the file  'Detector_merge.txt'\n";

    //Compare
    for (unsigned int p = 0; p < C.size(); p++)
      for (unsigned int i = 0; i < C[p].size(); i++)
      {
        //Search equal polygon values in curent part labels
        for (unsigned int k = 0; k < C[p].size(); k++)
          if (i != k) EXPECT_FALSE(C[p][i].getPolygon() == C[p][k].getPolygon()) << "partID = " << p << ": polygons of label_Num " << i << " and " << k << "is equal" << endl;
        vector<Score> LimbScores = C[p][i].getScores();
        //Search equal scores in curent label
        for (unsigned int k = 0; k < LimbScores.size(); k++)
          for (unsigned int t = 0; t < LimbScores.size(); t++)
            if (t != k) EXPECT_FALSE(LimbScores[k] == LimbScores[t]) << ", PartID = " << p << ": Equal scores in label with Num = " << i << endl;
      }
  }

  // Unit test for 'merge' function with statically generated data.
  // This unit test checks only scores
  TEST(DetectorTests, merge_StaticData)
  {
    map <uint32_t, vector <LimbLabel>> a;

    //Create polygons
    const int polygonsCount = 10;
    vector<vector<Point2f>> Polygons;
    for (int i = 0; i < polygonsCount; i++)
      Polygons.push_back(vector < Point2f > { Point2f(static_cast<float>(i), static_cast<float>(i)) });

    const int partCount = 2;
    const int a_size = 10;

    for (int p = 0; p < partCount; p++)
    {
      vector<LimbLabel> temp_partLabels;
      for (int i = 0; i < a_size; i++)
      {
        float n = static_cast<float>(i);
        LimbLabel temp_label(p, Point2f(n, n), n, Polygons[i], vector < Score > { Score(i / (float)10, "a", 1.0f)});
        temp_partLabels.push_back(temp_label);
      }
      a.insert(pair<uint32_t, vector <LimbLabel>>(p, temp_partLabels));
    }
    ColorHistDetector d;
    auto b = d.merge(a, a, map<uint32_t, vector<LimbLabel>>());

    EXPECT_EQ(a.size(), b.size());

    for (unsigned int p = 0; p < a.size(); p++)
    {
      vector <LimbLabel> temp = b[p];
      EXPECT_EQ(temp.size(), a_size);
      for (unsigned int i = 0; i < temp.size(); i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 1);
        EXPECT_EQ(scores[0].getScore(), a[p][i].getScores().at(0).getScore());
        EXPECT_EQ(scores[0].getDetName(), a[p][i].getScores().at(0).getDetName());
      }
    }

    map <uint32_t, vector <LimbLabel>> c;
    c = d.merge(b, c, map<uint32_t, vector<LimbLabel>>());

    for (unsigned int p = 0; p < a.size(); p++)
    {
      vector <LimbLabel> temp = c[p];
      EXPECT_EQ(temp.size(), a_size);
      for (unsigned int i = 0; i < temp.size(); i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 1);
        EXPECT_EQ(scores[0].getScore(), a[p][i].getScores().at(0).getScore());
        EXPECT_EQ(scores[0].getDetName(), a[p][i].getScores().at(0).getDetName());
      }
    }
    map <uint32_t, vector <LimbLabel>> g;
    g = d.merge(g, c, map<uint32_t, vector<LimbLabel>>());

    for (unsigned int p = 0; p < a.size(); p++)
    {
      vector <LimbLabel> temp = g[p];
      EXPECT_EQ(temp.size(), a_size);
      for (unsigned int i = 0; i < temp.size(); i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 1);
        EXPECT_EQ(scores[0].getScore(), a[p][i].getScores().at(0).getScore());
        EXPECT_EQ(scores[0].getDetName(), a[p][i].getScores().at(0).getDetName());
      }
    }

    map <uint32_t, vector <LimbLabel>> e;

    for (int p = 0; p < partCount; p++)
    {
      vector<LimbLabel> temp_partLabels;
      for (int i = 0; i < a_size; i++)
      {
        float n = static_cast<float>(i);
        LimbLabel temp_label(p, Point2f(n, n), n, Polygons[i], vector < Score > { Score(n / (float)10, "e", 1.0f)});
        temp_partLabels.push_back(temp_label);
      }
      e.insert(pair<uint32_t, vector <LimbLabel>>(p, temp_partLabels));
    }

    auto f = d.merge(c, e, map<uint32_t, vector<LimbLabel>>());

    for (unsigned int p = 0; p < a.size(); p++)
    {
      vector <LimbLabel> temp = f[p];
      EXPECT_EQ(temp.size(), a_size);
      for (unsigned int i = 0; i < temp.size(); i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 2);
        EXPECT_EQ(scores[0].getScore(), c[p][i].getScores().at(0).getScore());
        EXPECT_EQ(scores[0].getDetName(), c[p][i].getScores().at(0).getDetName());
        EXPECT_EQ(scores[1].getScore(), e[p][i].getScores().at(0).getScore());
        EXPECT_EQ(scores[1].getDetName(), e[p][i].getScores().at(0).getDetName());
      }
    }

    map <uint32_t, vector <LimbLabel>> h;
    map <uint32_t, vector <LimbLabel>> m;

    for (int p = 0; p < partCount; p++)
    {
      vector<LimbLabel> temp_partLabels;
      for (int i = 0; i < a_size / 2; i++)
      {
        float n = static_cast<float>(i);
        LimbLabel temp_label(p, Point2f(n, n), n, Polygons[i], vector < Score > { Score(n / (float)10, "h", 1.0f)});
        temp_partLabels.push_back(temp_label);
      }
      h.insert(pair <uint32_t, vector <LimbLabel>>(p, temp_partLabels));
    }

    for (int p = 0; p < partCount; p++)
    {
      vector<LimbLabel> temp_partLabels;
      for (int i = a_size / 2; i < a_size; i++)
      {
        float n = static_cast<float>(i);
        LimbLabel temp_label(p, Point2f(n, n), n, Polygons[i], vector < Score > { Score(n / (float)10, "m", 1.0f)});
        temp_partLabels.push_back(temp_label);
      }
      m.insert(pair <uint32_t, vector <LimbLabel>>(p, temp_partLabels));
    }

    auto n = d.merge(h, m, map<uint32_t, vector<LimbLabel>>());

    for (unsigned int p = 0; p < a.size(); p++)
    {
      vector <LimbLabel> temp = n[p];
      EXPECT_EQ(temp.size(), a_size);
      for (unsigned int i = 0; i < temp.size() / 2; i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 2);
        bool bFound = false;
        LimbLabel o;
        for (unsigned int k = 0; k < h[p].size(); k++)
        {
          if (l.getPolygon() == h[p][k].getPolygon())
          {
            bFound = true;
            o = h[p][k];
            break;
          }
        }
        EXPECT_TRUE(bFound);
        if (bFound)
        {
          EXPECT_EQ(scores[0].getScore(), o.getScores().at(0).getScore());
          EXPECT_EQ(scores[0].getDetName(), o.getScores().at(0).getDetName());
          EXPECT_EQ(scores[1].getScore(), 1.0);
          EXPECT_EQ(scores[1].getDetName(), m[p][0].getScores().at(0).getDetName());
        }
      }

      for (unsigned int i = temp.size() / 2; i < temp.size(); i++)
      {
        LimbLabel l = temp[i];
        vector <Score> scores = l.getScores();
        EXPECT_EQ(scores.size(), 2);
        bool bFound = false;
        LimbLabel q;
        for (unsigned int k = 0; k < m[p].size(); k++)
        {
          if (l.getPolygon() == m[p][k].getPolygon())
          {
            bFound = true;
            q = m[p][k];
            break;
          }
        }
        EXPECT_TRUE(bFound);
        if (bFound)
        {
          EXPECT_EQ(scores[0].getScore(), q.getScores().at(0).getScore());
          EXPECT_EQ(scores[0].getDetName(), q.getScores().at(0).getDetName());
          EXPECT_EQ(scores[1].getScore(), 1.0);
          EXPECT_EQ(scores[1].getDetName(), h[p][0].getScores().at(0).getDetName());
        }
      }
    }

  }

  TEST(DetectorTests, getBodyPartRect)
  {
    //Prepare input data
    vector<Point2f> p0; // parent joints locations/begin of each from parts vectors
    vector<Point2f> p1; // child joints locations/end of each from parts vectors
    p0.push_back(Point2f(1.0f, 1.0f)); p1.push_back(Point2f(1.2f, 0.5f));
    p0.push_back(Point2f(2.0f, 5.0f)); p1.push_back(Point2f(5.0f, 2.0f));
    p0.push_back(Point2f(5.0f, 2.0f)); p1.push_back(Point2f(2.0f, 5.0f));
    p0.push_back(Point2f(5.0f, -5.0f)); p1.push_back(Point2f(2.0f, -2.0f));
    p0.push_back(Point2f(-5.0f, -5.0f)); p1.push_back(Point2f(-2.0f, -2.0f));
    p0.push_back(Point2f(5.0f, 1.0f)); p1.push_back(Point2f(0.0f, 0.0f));
    p0.push_back(Point2f(1.0f, 5.0f)); p1.push_back(Point2f(0.0f, 0.0f));
    p0.push_back(Point2f(5.0f, 0.0f)); p1.push_back(Point2f(1.0f, 0.0f));
    float LWRatio = 1.5;
    BodyJoint joint0(0, "j0", p0[0], Point3f(p0[0]), false);
    BodyJoint joint1(1, "j0", p1[0], Point3f(p1[0]), false);
    BodyPart part(0, "part1", 0, 1, false, 0);
    part.setLWRatio(LWRatio);
   
    //Create expected value (used alternative function from "TestsFunctions.cpp")
    /*
    vector<Point2f> rect;
    Point2f d = p0 - p1;
    float L = sqrt(pow(p1.x - p0.x,2) + pow(p1.y - p0.y, 2));
    float k = 0.5f/LWRatio;
    float dx = k*(d.y);
    float dy = k*(-d.x);
    rect.push_back(Point2f(p0.x + dx, p0.y + dy));
    rect.push_back(Point2f(p1.x + dx, p1.y + dy));
    rect.push_back(Point2f(p1.x - dx, p1.y - dy));
    rect.push_back(Point2f(p0.x - dx, p0.y - dy));
    */
    vector <vector<Point2f>> partsRects_expected;
    for (unsigned int i = 0; i < p0.size(); i++)
    {
      vector<Point2f> temp = getPartRect(LWRatio, p0[i], p1[i]);
      partsRects_expected.push_back(temp);
      //temp.clear();
    }

    //Craete actual value
    vector <vector<Point2f>> partsRects_actual;
    ColorHistDetector D;
    for (unsigned int i = 0; i < p0.size(); i++)
    {
      //joint0.setImageLocation(p0[i]);
      //joint0.setSpaceLocation(Point3f(p0[i]));
      //joint1.setImageLocation(p1[i]);
      //joint1.setSpaceLocation(Point3f(p1[i]));
      spelRECT<cv::Point2f> partRect = part.getBodyPartRect(p0[i], p1[i]);
      partsRects_actual.push_back(partRect.asVector());
    }

    //Compare
    //EXPECT_EQ(partsRects_expected, partsRects_actual);
    float error = 0.000001f;
    for (unsigned int i = 0; i < partsRects_expected.size(); i++)
    {
      //cout << i << ": " << "Expected ~ Actual" << endl;
      for (unsigned int k = 0; k < partsRects_expected[i].size(); k++)
      {
        //cout << partsRects_expected[i][k] << "~" << partsRects_actual[i][k] << endl;
        //EXPECT_FLOAT_EQ(partsRects_expected[i][k].x, partsRects_actual[i][k].x);
        //EXPECT_FLOAT_EQ(partsRects_expected[i][k].y, partsRects_actual[i][k].y);
        EXPECT_NEAR(partsRects_expected[i][k].x, partsRects_actual[i][k].x, error);
        EXPECT_NEAR(partsRects_expected[i][k].y, partsRects_actual[i][k].y, error);
      }

      partsRects_expected[i].clear();
      partsRects_actual[i].clear();
    }

    partsRects_expected.clear();
    partsRects_actual.clear();
  }

  TEST(DetectorTests, getBodyPartRect_withBlockSize)
  {
    //Prepare input data
    //Point2f p0 = Point2f(50.0f, 20.0f); // parent joint location/begin of the part vector
    //Point2f p1 = Point2f(20.0f, 50.0f); // child joint location/end of ethe part vector
    //Point2f p0 = Point2f(20.0f, 50.0f); // parent joint location/begin of the part vector
    //Point2f p1 = Point2f(50.0f, 20.0f); // child joint location/end of ethe part vector
    //Point2f p0 = Point2f(50.0f, 50.0f); // parent joint location/begin of the part vector
    //Point2f p1 = Point2f(10.0f, 50.0f); // child joint location/end of the part vector
    Point2f p0 = Point2f(10.0f, 50.0f); // parent joint location/begin of the part vector
    Point2f p1 = Point2f(50.0f, 50.0f); // child joint location/end of the part vector

    float LWRatio = 1.5;
    BodyJoint joint0(0, "j0", p0, Point3f(p0), false);
    BodyJoint joint1(1, "j0", p1, Point3f(p1), false);
    BodyPart part(0, "part1", 0, 1, false, 0);
    part.setLWRatio(LWRatio);
   
    //Create expected value (used alternative function from "TestsFunctions.cpp")
    vector <vector<Point2f>> partsRect_expected;
    vector<Point2f> partRect_expected = getPartRect(LWRatio, p0, p1, cv::Size(8,8));

    //Craete actual value
    ColorHistDetector D;
    vector<Point2f> partRect_actual = part.getBodyPartRect(p0, p1, cv::Size(8, 8)).asVector();

    Mat img1 = Mat(Size(100, 100), CV_8UC3, Scalar(255, 255, 255));
    PutPartRect(img1, partRect_actual, Scalar(0, 0, 0));
    PutPartRect(img1, partRect_expected, Scalar(0, 0, 255));
    line(img1, p0, p1, Scalar(255, 0, 0), 1, 1);
    imwrite("detector_getBodyPartRect.jpg", img1);

    //spelRECT<cv::Point2f> partRect = D.getBodyPartRect(part, p0, p1);
    //vector<Point2f> partRect_actual = partRect.asVector();
    cout << "Expected part rect:" << endl << partRect_expected << endl;
    cout << "Actual part rect:"<< endl << partRect_actual << endl;

    cout << "Block size: 8 x 8" << endl;
    cout << "Initial part size: ";
    vector<Point2f> partRect = getPartRect(LWRatio, p0, p1);
    cout << sqrt(pow(partRect[0].x - partRect[1].x, 2) + pow(partRect[0].y - partRect[1].y, 2)) << " x ";
    cout << sqrt(pow(partRect[0].x - partRect[3].x, 2) + pow(partRect[0].y - partRect[3].y, 2)) << endl;
    cout << "Expected part size: ";
    cout << sqrt(pow(partRect_expected[0].x - partRect_expected[1].x, 2) + pow(partRect_expected[0].y - partRect_expected[1].y, 2)) << " x ";
    cout << sqrt(pow(partRect_expected[0].x - partRect_expected[3].x, 2) + pow(partRect_expected[0].y - partRect_expected[3].y, 2)) << endl;
    cout << "Actual part size: ";
    cout << sqrt(pow(partRect_actual[0].x - partRect_actual[1].x, 2) + pow(partRect_actual[0].y - partRect_actual[1].y, 2)) << " x ";
    cout << sqrt(pow(partRect_actual[0].x - partRect_actual[3].x, 2) + pow(partRect_actual[0].y - partRect_actual[3].y, 2)) << endl << endl;

    EXPECT_EQ(partRect_expected, partRect_actual);
    partRect_expected.clear();
    partRect_actual.clear();
  }

  TEST(DetectorTests, detect)
  {
    //Prepare test data
    map<string, float> params;
    TestSequence sequence(params, "speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> frames = sequence.getFrames();
    //sequence.TestSequence::~TestSequence();

    //Run "train"
    ColorHistDetector detector;
    detector.train(frames, params);
    Skeleton skeleton = frames[0]->getSkeleton(); // Copy skeleton from keyframe
    frames[1]->setSkeleton(skeleton); // Set skeleton from keyframe to frames[1] 

    //Run "detect"
    map<uint32_t, vector<LimbLabel>> limbLabels;
    ColorHistDetectorHelper* detectorHelper = new ColorHistDetectorHelper();
    std::map <int32_t, cv::Mat> pixelDistributions = detector.buildPixelDistributions(frames[0]);
    detectorHelper->pixelLabels = detector.buildPixelLabels(frames[0], pixelDistributions);
    limbLabels = detector.Detector::detect(frames[1], params, limbLabels, detectorHelper);
    ASSERT_GT(limbLabels.size(), 0);

    // Output top of "limbLabels" into text file
    ofstream fout("Output_DetectorTest_detect.txt");
    fout << "\nTop Labels, sorted by part id:\n\n";
    for (unsigned int i = 0; i < limbLabels.size(); i++) // For all body parts
    {
      for (unsigned int k = 0; (k < limbLabels[i].size()) && (k < 4); k++) // For all scores of this bodypart
      {
        Point2f p0, p1;
        limbLabels[i][k].getEndpoints(p0, p1); // Copy the Limblabel points
        fout << "  " << i << ":" << " limbID = " << limbLabels[i][k].getLimbID() << ", Angle = " << limbLabels[i][k].getAngle() << ", Points = {" << p0 << ", " << p1 << "}, AvgScore = " << limbLabels[i][k].getAvgScore() << ", Scores = {";
        vector<Score> scores = limbLabels[i][k].getScores(); // Copy the Label scores
        for (unsigned int t = 0; t < scores.size(); t++)
        {
          fout << scores[t].getScore() << ", "; // Put all scores of the Label
        }
        fout << "}\n";
      }
      fout << endl;
    }

    // Copy coordinates of BodyParts from skeleton
    map<int, pair<Point2f, Point2f>> PartLocation = getPartLocations(skeleton);

    // Compare labels with ideal bodyparts from keyframe, and output debug information 
    float TolerableCoordinateError = 7.0f; // Linear error in pixels
    //float TolerableAngleError = 0.1f; // 10% (not used in this test)
    int TopListLabelsCount = 4; // Size of "labels top list"
    map<int, vector<LimbLabel>> effectiveLabels;
    vector<int> WithoutGoodLabelInTop;
    bool EffectiveLabbelsInTop = true;

    fout << "-------------------------------------\nAll labels, with distance from the ideal body part: \n";

    for (unsigned int id = 0; id < limbLabels.size(); id++)
    {
      fout << "\nPartID = " << id << ":\n";
      Point2f l0, l1, p0, p1, delta0, delta1;
      vector<LimbLabel> temp;
      p0 = PartLocation[id].first; // Ideal body part point
      p1 = PartLocation[id].second; // Ideal body part point
      for (int k = 0; k < static_cast<int>(limbLabels[id].size()); k++)
      {
        limbLabels[id][k].getEndpoints(l0, l1); // Label points
        delta0 = l0 - p0;
        delta1 = l1 - p1;
        float error_A = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
        delta0 = l0 - p1;
        delta1 = l1 - p0;
        float error_B = max(sqrt(pow(delta0.x, 2) + pow(delta0.y, 2)), sqrt(pow(delta1.x, 2) + pow(delta1.y, 2)));
        float error = min(error_A, error_B); // Distance between ideal body part and label
        if (error <= TolerableCoordinateError && limbLabels[id][k].getAvgScore() >= 0) // Label is "effective" if it has small error and of not less than zero  Score  value
          temp.push_back(limbLabels[id][k]); // Copy effective labels
        // Put linear errors for all Labels into text file, copy indexes of a "badly processed parts"
        fout << "    PartID = " << id << ", LabelIndex = " << k << ":    AvgScore = " << limbLabels[id][k].getAvgScore() << ", LinearError = " << error << endl;
        if (k == TopListLabelsCount - 1)
        {
          fout << "    //End of part[" << id << "] top labels list\n";
          if (!(skeleton.getBodyPart(id)->getIsOccluded()))
            if (temp.size() < 1)
            {
              EffectiveLabbelsInTop = false; // false == Present not Occluded bodyparts, but no nave "effective labels" in the top of list
              WithoutGoodLabelInTop.push_back(id); // Copy index of not Occluded parts, which no have "effective labels" in the top of labels list
            }
        }
      }
      effectiveLabels.emplace(pair<int, vector<LimbLabel>>(id, temp));
    }

    //Output top of "effectiveLabels" into text file
    fout << "\n-------------------------------------\n\nTrue Labels:\n\n";
    for (unsigned int i = 0; i < effectiveLabels.size(); i++)
    {
      for (unsigned int k = 0; k < effectiveLabels[i].size(); k++)
      {
        Point2f p0, p1;
        limbLabels[i][k].getEndpoints(p0, p1);
        fout << "  limbID = " << effectiveLabels[i][k].getLimbID() << ", Angle = " << effectiveLabels[i][k].getAngle() << ", Points = {" << p0 << ", " << p1 << "}, AvgScore = " << effectiveLabels[i][k].getAvgScore() << ", Scores = {";
        vector<Score> scores = effectiveLabels[i][k].getScores();
        for (unsigned int t = 0; t < scores.size(); t++)
        {
          fout << scores[t].getScore() << ", ";
        }

        fout << "}\n";
      }
      fout << endl;
    }

    fout.close();
    cout << "\nLimbLabels saved in file: Output_DetectorTest_detect.txt\n";

    // Output messages 
    if (!EffectiveLabbelsInTop) cout << endl << "Detector_Tests.detect:" << endl;
    EXPECT_TRUE(EffectiveLabbelsInTop);
    if (!EffectiveLabbelsInTop)
    {
      cout << "Body parts with id: ";
      for (unsigned int i = 0; i < WithoutGoodLabelInTop.size(); i++)
      {
        cout << WithoutGoodLabelInTop[i];
        if (i != WithoutGoodLabelInTop.size() - 1) cout << ", ";
      }
      cout << " - does not have effective labels in the top of labels list." << endl;
    }
    if (!EffectiveLabbelsInTop) cout << endl;

    // Clear
    for (int i = 0; i < frames.size(); i++)
      delete frames[i];
    frames.clear();
  }

  TEST(DetectorTests, generateLabel1)
  {
    // Prepare test data
    int part_id = 0, parentJoint_id = 0, childJoint_id = 1;
    Point2f p0(30.0f, 10.0f), p1(60.0f, 10.0f);
    Point2f d = p1 - p0;
    float Length = sqrt(d.x*d.x + d.y*d.y);
    Point2f shift(0.5f*d.x, 0);
    bool isOccluded = false;
    BodyPart* bodyPart0 = new BodyPart(part_id, "", parentJoint_id, childJoint_id, isOccluded, Length);
    BodyPart* bodyPart1 = new BodyPart(part_id+1, "", parentJoint_id, childJoint_id, isOccluded, Length);
    float LWRatio = 3.0f;
    bodyPart0->setLWRatio(LWRatio);
    bodyPart1->setLWRatio(LWRatio);
    //BodyJoint j0(parentJoint_id, "", p0, Point3f(p0.x, p0.y, 0.0f), false);
    //BodyJoint j1(parentJoint_id, "", p1, Point3f(p1.x, p1.y, 0.0f), false);

    ColorHistDetector detector(8);
    ColorHistDetector::PartModel model;
    model.fgNumSamples = 1;
    model.fgSampleSizes.push_back(1);
    detector.partModels.emplace(pair<int32_t, ColorHistDetector::PartModel>(part_id, model));
    detector.partModels.emplace(pair<int32_t, ColorHistDetector::PartModel>(part_id+1, model));
    ColorHistDetectorHelper* detectorHelper = new ColorHistDetectorHelper();


    int rows = 100, cols = 140;
    Mat Image = Mat::zeros(rows, cols, CV_8UC3);
    Mat Mask = Mat::zeros(rows, cols, CV_8UC1);
    Mat pixelsLabels = Mat(rows, cols, cv::DataType <float>::type, 0.0f);
    Mat ShiftedixelsLabels = Mat(rows, cols, cv::DataType <float>::type, 0.0f);

    vector<Point2f> rect = getPartRect(LWRatio, p0, p1);

    for (int x = 0; x < cols; x++)
      for (int y = 0; y < rows; y++)
        if(pointPolygonTest(rect, Point2f(x, y), false) > 0)
        {
          Image.at<Vec3b>(y, x) = Vec3b(255, 255, 255);
          pixelsLabels.at<float>(y, x) = 1.0f;
          ShiftedixelsLabels.at<float>(y + shift.y, x + shift.x) = 1.0f;
        }
    detectorHelper->pixelLabels.emplace(pair<int32_t, cv::Mat>(part_id, pixelsLabels));
    detectorHelper->pixelLabels.emplace(pair<int32_t, cv::Mat>(part_id+1, ShiftedixelsLabels));
    cvtColor(Image, Mask, CV_BGR2GRAY);

    Frame* frame0 = new Lockframe();
    frame0->setID(0);
    frame0->setImage(Image);
    frame0->setMask(Mask);

    BodyPart * bodyPart = bodyPart0;
    auto comparer = [&]() -> float
    {
      return detector.compare(*bodyPart, frame0, detectorHelper->pixelLabels, p0, p1);
    };

    auto comparer1 = [&]() -> float
    {
      return detector.compare(*bodyPart, frame0, detectorHelper->pixelLabels, (p0 - shift), (p1 - shift));
    };
     
    // Create actual value
    float usedet = 1.0f;
    LimbLabel label = detector.Detector::generateLabel(*bodyPart0, p0, p1, "", usedet, comparer);// Part rect, mask and PixelsLabels fully coincide
    cout << "label score = " << label.getAvgScore() << endl;
    
    cout << "Rect shift = " << shift << ", ";
    LimbLabel shiftedLabel = detector.Detector::generateLabel(*bodyPart0, (p0 - shift), (p1 - shift), "", usedet, comparer1);// Part rect shifted
    cout << "shifted rect score = " << shiftedLabel.getAvgScore() << endl;
  
    cout << "PixelsLabels shift = " << - shift << ", ";
    bodyPart = bodyPart1;
    LimbLabel shiftedPLabels_Label = detector.Detector::generateLabel(*bodyPart1, p0, p1, "", usedet, comparer);// PixelsLabels shifted
    cout << "label with shifted pixelsLabels score = " << shiftedPLabels_Label.getAvgScore() << endl << endl;

    // Compare
    float error = 0.05f;
    EXPECT_EQ(0.0f, label.getAvgScore());
    EXPECT_NEAR(1.0f - (0.5f*0.5f + 0.5f*1.0f), shiftedLabel.getAvgScore(), error);
    EXPECT_NEAR(1.0f - 0.5f*(0.5f + 0.5f), shiftedPLabels_Label.getAvgScore(), error);

    EXPECT_EQ(part_id, label.getLimbID());
    EXPECT_EQ(part_id, shiftedLabel.getLimbID());
    EXPECT_EQ(part_id + 1, shiftedPLabels_Label.getLimbID());
    
    EXPECT_EQ(0.5f*(p0 + p1), label.getCenter());
    EXPECT_EQ(0.5f*(p0 + p1) - shift, shiftedLabel.getCenter());
    EXPECT_EQ(0.5f*(p0 + p1), shiftedPLabels_Label.getCenter());

    //error = 0.00001f;
    //float angle = spelHelper::angle2D(p0.x, p0.y, p1.x, p1.y);
    //EXPECT_NEAR(angle, label.getAngle(), error);
    //EXPECT_NEAR(angle, shiftedLabel.getAngle(), error);
    //EXPECT_NEAR(angle, shiftedPLabels_Label.getAngle(), error);
    //Angle: expected angle2D(p0.x, p0.y, p1.x, p1.y) = -0.157, Actual label.getAngle() = 0. All must be 0.

    // Clear
    Image.release();
    Mask.release();
    pixelsLabels.release();
    ShiftedixelsLabels.release();
    delete frame0;
  }

 TEST(DetectorTests, generateLabel2)
  {
    // Prepare test data
    int part_id = 0, parentJoint_id = 0, childJoint_id = 1;
    Point2f p0(30.0f, 10.0f), p1(60.0f, 10.0f);
    Point2f center = 0.5f*(p0 + p1);
    float angle = spelHelper::angle2D(p0.x, p0.y, p1.x, p1.y);
    Point2f d = p1 - p0;
    float length = sqrt(d.x*d.x + d.y*d.y);
    Point2f shift(0.5f*d.x, 0.5*d.y);
    bool isOccluded = false;
    BodyPart bodyPart0(part_id, "", parentJoint_id, childJoint_id, isOccluded, length);
    BodyPart bodyPart1(part_id+1, "", parentJoint_id, childJoint_id, isOccluded, length);
    float LWRatio = 3.0f;
    bodyPart0.setLWRatio(LWRatio);
    bodyPart1.setLWRatio(LWRatio);
    //BodyJoint j0(parentJoint_id, "", p0, Point3f(p0.x, p0.y, 0.0f), false);
    //BodyJoint j1(parentJoint_id, "", p1, Point3f(p1.x, p1.y, 0.0f), false);

    ColorHistDetector detector(8);
    ColorHistDetector::PartModel model;
    model.fgNumSamples = 1;
    model.fgSampleSizes.push_back(1);
    detector.partModels.emplace(pair<int32_t, ColorHistDetector::PartModel>(part_id, model));
    detector.partModels.emplace(pair<int32_t, ColorHistDetector::PartModel>(part_id+1, model));
    ColorHistDetectorHelper* detectorHelper = new ColorHistDetectorHelper();


    int rows = 100, cols = 140;
    Mat Image = Mat::zeros(rows, cols, CV_8UC3);
    Mat Mask = Mat::zeros(rows, cols, CV_8UC1);
    Mat pixelsLabels = Mat(rows, cols, cv::DataType <float>::type, 0.0f);
    Mat ShiftedixelsLabels = Mat(rows, cols, cv::DataType <float>::type, 0.0f);

    vector<Point2f> rect = getPartRect(LWRatio, p0, p1);

    for (int x = 0; x < cols; x++)
      for (int y = 0; y < rows; y++)
        if(pointPolygonTest(rect, Point2f(x, y), false) > 0)
        {
          Image.at<Vec3b>(y, x) = Vec3b(255, 255, 255);
          pixelsLabels.at<float>(y, x) = 1.0f;
          ShiftedixelsLabels.at<float>(y + shift.y, x + shift.x) = 1.0f;
        }
    detectorHelper->pixelLabels.emplace(pair<int32_t, cv::Mat>(part_id, pixelsLabels));
    detectorHelper->pixelLabels.emplace(pair<int32_t, cv::Mat>(part_id+1, ShiftedixelsLabels));
    cvtColor(Image, Mask, CV_BGR2GRAY);

    Frame* frame0 = new Lockframe();
    frame0->setID(0);
    frame0->setImage(Image);
    frame0->setMask(Mask);
     
    // Create actual value
    map<string, float> params;
    LimbLabel label = detector.Detector::generateLabel(length, angle, center.x, center.y, bodyPart0, frame0, detectorHelper, params);// Part rect, mask and PixelsLabels fully coincide
    cout << "label score = " << label.getAvgScore() << endl;
    
    cout << "Rect shift = " << shift << ", ";
    LimbLabel shiftedLabel = detector.Detector::generateLabel(length, angle, center.x  + shift.x, center.y + shift.y, bodyPart0, frame0, detectorHelper, params);// Part rect shifted
    cout << "shifted rect score = " << shiftedLabel.getAvgScore() << endl;
 
    cout << "PixelsLabels shift = " << shift << ", ";
    LimbLabel shiftedPLabels_Label = detector.Detector::generateLabel(length, angle, center.x, center.y, bodyPart1, frame0, detectorHelper, params);// PixelsLabels shifted
    cout << "label with shifted pixelsLabels score = " << shiftedPLabels_Label.getAvgScore() << endl << endl;

    // Compare
    float error = 0.05f;
    EXPECT_NEAR(0.0f, label.getAvgScore(), error);
    EXPECT_NEAR(1.0f - (0.5f*0.5f + 0.5f*1.0f), shiftedLabel.getAvgScore(), error);
    EXPECT_NEAR(1.0f - 0.5f*(0.5f + 0.5f), shiftedPLabels_Label.getAvgScore(), error);

    EXPECT_EQ(part_id, label.getLimbID());
    EXPECT_EQ(part_id, shiftedLabel.getLimbID());
    EXPECT_EQ(part_id + 1, shiftedPLabels_Label.getLimbID());
    
    EXPECT_EQ(0.5f*(p0 + p1), label.getCenter());
    EXPECT_EQ(0.5f*(p0 + p1) + shift, shiftedLabel.getCenter());
    EXPECT_EQ(0.5f*(p0 + p1), shiftedPLabels_Label.getCenter());

    error = 0.00001f;
    EXPECT_NEAR(angle, label.getAngle(), error);
    EXPECT_NEAR(angle, shiftedLabel.getAngle(), error);
    EXPECT_NEAR(angle, shiftedPLabels_Label.getAngle(), error);

    //vector<Point2f> polygon = getPartRect(LWRatio, p0, p1);
    //EXPECT_EQ(polygon, stlabel.getPolygon());
    //EXPECT_EQ(getPartRect(LWRatio, p0 + shift, p1 + shift), shiftedLabel.getPolygon());
    //EXPECT_EQ(polygon, shiftedPLabels_Label.getPolygon());

    Image.release();
    Mask.release();
    pixelsLabels.release();
    ShiftedixelsLabels.release();
    delete frame0;
  }

  bool CompareLabels (LimbLabel X, LimbLabel Y)
  {
      return Y.getAvgScore() > X.getAvgScore();
  }

 TEST(DetectorTests, FilterLimbLabels)
 {
   // Create expected value
   vector<LimbLabel> labels;
   int N = 2, M = 2;
   float P = 1.0f*N / (N + M);
   int n = 3, m = 4, p = 3;
   float d = 0.1f;
   int id = 0;
   for (int t = 0; t < N; t++)
     for (int i = 0; i < n; i++)
       for (int k = 0; k < m; k++)
        for (int l = 0; l < p; l++)
        { 
          Score score(0.4f*rand()/RAND_MAX,"", 1.0f);
          vector<Score> scores;
          scores.push_back(score);
          vector<Point2f> polygon;
          labels.push_back(LimbLabel(id, Point2f(i*d, k*d), l*d, polygon, scores, false));
          id++;
        }
   sort(labels.begin(), labels.end(), CompareLabels);

   // Prepare test data
   vector<LimbLabel> temp(labels.size());
   copy(labels.begin(), labels.end(), temp.begin());
   for (int t = 0; t < M; t++)
     for (int i = 0; i < n; i++)
       for (int k = 0; k < m; k++)
         for (int l = 0; l < p; l++)
         {
           Score score(0.5f + 0.4f*rand() / RAND_MAX, "", 1.0f);
           vector<Score> scores;
           scores.push_back(score);
           vector<Point2f> polygon;
           temp.push_back(LimbLabel(id, Point2f(i*d, k*d), l*d, polygon, scores, false));
           id++;
         }
   sort(temp.begin(), temp.end(), CompareLabels);

   // Create actual value
   ColorHistDetector detector(8);
   vector<LimbLabel> labels_actual;
   labels_actual = detector.filterLimbLabels(temp, P, P);
   
   ASSERT_EQ(labels.size(), labels_actual.size());
   for (int i = 0; i < labels.size(); i++)
     EXPECT_EQ(labels[i].getLimbID(), labels_actual[i].getLimbID());

   /*
   for (int i = 0; i < min(labels.size(), labels_actual.size()); i++)
     cout << i << ": " << labels[i].getAvgScore() <<" ~ " << actual_labels[i].getAvgScore() << endl;
   */
 }

  TEST(DetectorTests, FilterLimbLabels_B)
 {
   // Prepare test data
   vector<LimbLabel> labels_expected;
   vector<LimbLabel> temp;
   int n = 3, m = 4, p = 3;
   float d = 0.1f;
   int id = 0;
   cout << "Test data:" << endl;
   for (int i = 1; i < n+1; i++)
     for (int k = 1; k < m+1; k++)
     { 
       Score score1(1.0f*id/100.0f,"", 1.0f);
       Score score2(1.0f*id/100.0f + 0.001f, "", 1.0f);
       vector<Score> scores;
       scores.push_back(score1);
       vector<Point2f> polygon;
       labels_expected.push_back(LimbLabel(id, Point2f(i*d, k*d), d, polygon, scores, false));
       cout << id << ": Score = " << score1.getScore() << " {Center = " << labels_expected.back().getCenter() << ", Angle = " << labels_expected.back().getAngle() << "}" << endl;
       id++;
       scores.clear();
       scores.push_back(score2);
       temp.push_back(LimbLabel(id, Point2f(i*d, k*d), d, polygon, scores, false));
       cout << id << ": Score = " << score2.getScore() << " {Center = " << temp.back().getCenter() << ", Angle = " << labels_expected.back().getAngle() << "}" << endl;
       id++;   

       scores.clear();
     }
   cout << "Vector size = " << labels_expected.size() + temp.size() << endl;

   for (int l = 1; l < p+1; l++)
   { 
     Score score1(1.0f*id/100.0f,"", 1.0f);
     Score score2(1.0f*id/100.0f + 0.001f, "", 1.0f);
     vector<Score> scores;
     scores.push_back(score1);
     vector<Point2f> polygon;
     labels_expected.push_back(LimbLabel(id, Point2f(d, d), l*d, polygon, scores, false));
     id++;
     cout << id << ": Score = " << score1.getScore() << " {Center = " << labels_expected.back().getCenter() << ", Angle = " << labels_expected.back().getAngle() << "}" << endl;
     scores.clear();
     scores.push_back(score2);
     temp.push_back(LimbLabel(id, Point2f(d, d), l*d, polygon, scores, false));
     id++;
     cout << id << ": Score = " << score2.getScore() << " {Center = " << labels_expected.back().getCenter() << ", Angle = " << labels_expected.back().getAngle() << "}" << endl;
     
     scores.clear();
   }
   //cout << "Vector size = " << labels_expected.size() + temp.size() << endl;
   sort(labels_expected.begin(), labels_expected.end(), CompareLabels);
   sort(temp.begin(), temp.end(), CompareLabels);

   vector<LimbLabel> temp2(2* labels_expected.size());
   merge(labels_expected.begin(), labels_expected.end(), temp.begin(), temp.end(), temp2.begin());
   cout << "Vector size = " << temp2.size() << endl;

   // Create expected value
   int N = labels_expected.size();
   labels_expected.resize(N + 6);
   copy(temp.begin(), temp.begin() + 6, labels_expected.begin() + N);
   labels_expected.push_back(temp[12]);
   sort(labels_expected.begin(), labels_expected.end(), CompareLabels);

   // Create actual value
   ColorHistDetector detector(8);
   vector<LimbLabel> labels_actual;
   labels_actual = detector.filterLimbLabels(temp2, 0.5f, 0.5f);

   cout << "Expected size = " << labels_expected.size() << endl;
   cout << "Actual size = " << labels_actual.size() << endl;

   //Put result
   cout << endl << "Expected ~ Actual" << endl;
   for (int i = 0; i < min(labels_expected.size(), labels_actual.size()); i++)
   {
     cout << i << ": Score = " << labels_expected[i].getAvgScore() << " {Center = " << labels_expected[i].getCenter() <<
       ", Angle = " << labels_expected[i].getAngle() << "} ~ Score = " << labels_actual[i].getAvgScore() << " {Center = " << labels_actual[i].getCenter() <<
       ", Angle = " << labels_actual[i].getAngle() << "}" << endl;
   }

   ASSERT_EQ(labels_expected.size(), labels_actual.size());
   for (int id = 0; id < labels_expected.size(); id++)
     EXPECT_EQ(labels_expected[id].getLimbID(), labels_actual[id].getLimbID()) << endl;
 }

}
