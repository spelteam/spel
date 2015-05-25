#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif
#include <gtest/gtest.h>
#include <detector.hpp>
#include <colorHistDetector.hpp>

using namespace cv;

class TestingDetector : public ColorHistDetector
{
  public:
  Mat DeRotate(Mat imgSource, POSERECT <Point2f> &initialRect, float angle, Size size);
  float GetBoneLength(Point2f begin, Point2f end);
  float GetBoneWidth(float length, BodyPart bodyPart);
};

Mat TestingDetector::DeRotate(Mat imgSource, POSERECT <Point2f> &initialRect, float angle, Size size)
{
    return  TestingDetector::rotateImageToDefault(imgSource, initialRect, angle, size);
}

float TestingDetector::GetBoneLength(Point2f begin, Point2f end)
{
    return TestingDetector::getBoneLength(begin, end);
}

float TestingDetector::GetBoneWidth(float length, BodyPart bodyPart)
{
    return  TestingDetector::getBoneWidth(length, bodyPart);
}

POSERECT<Point2f> CreateRect(float x1, float x2, float y1, float y2)
{
    Point2f a(x1, y1), b(x2, y1), c(x2, y2), d(x1, y2), E(0, 0);
    POSERECT <Point2f> rect(a, b, c, d);
    return rect;
}

// Rotation of the rectangle around center 
POSERECT<Point2f> RotateRect(POSERECT<Point2f> &rect, float angle)
{
    POSERECT<Point2f> RotatedRect;
    Point2f center = rect.GetCenter<Point2f>();
    RotatedRect.point1 = PoseHelper::rotatePoint2D(rect.point1, center, angle);
    RotatedRect.point2 = PoseHelper::rotatePoint2D(rect.point2, center, angle);
    RotatedRect.point3 = PoseHelper::rotatePoint2D(rect.point3, center, angle);
    RotatedRect.point4 = PoseHelper::rotatePoint2D(rect.point4, center, angle);
    return RotatedRect;
}

// Filling the rectangle
void FillRect(Mat &Img, POSERECT<Point2f> &rect, Vec3b colour)
{
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    for (int i = (int)xmin; i <= (int)xmax; i++)
        for (int j = (int)ymin; j <= (int)ymax; j++)
            Img.at<Vec3b>(j, i) = colour;
}

void FillRotatedRect(Mat &Img, POSERECT<Point2f> &rect, Vec3b colour)
{
  float xmax, ymax, xmin, ymin;
  rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
  for (int i = (int)xmin; i <= (int)xmax; i++)
    for (int j = (int)ymin; j <= (int)ymax; j++)
      if (rect.containsPoint(Point2f((float)i,(float)j)) == 1)
        Img.at<Vec3b>(j, i) = colour;
}

void FillRectRand(Mat &Img, POSERECT<Point2f> &rect)
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
Mat RotateImage(Mat Img, POSERECT<Point2f> &rect, float angle)
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
        E = PoseHelper::rotatePoint2D(E, center, angle);
        if ((0 < E.x < size.width) && (0 < E.y < size.height))
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

TEST(DetectorTests, rotateImageToDefault_OneColor)
{
    // Prepare input data
    int rows = 200, cols = 200; // image size
    float angle = 45; // the rotation angle
    float x1 = 10.0, x2 = 40.0, y1 = 20.0, y2 = 60.0; // the rectangle vertices
    POSERECT <Point2f> rect = CreateRect(x1, x2, y1, y2);
    POSERECT <Point2f> RotatedRect = RotateRect(rect, angle);
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
    POSERECT <Point2f> rect = CreateRect(x1, x2, y1, y2);
    POSERECT <Point2f> RotatedRect = RotateRect(rect, angle);
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

    POSERECT <Point2f> RotatedRectCopy = RotatedRect;

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
    img1 = imread("Debug/posetests_TestData/ImageRotationTestData/image1.jpg");
    img2 = imread("Debug/posetests_TestData/ImageRotationTestData/image2.jpg");
    img3 = imread("Debug/posetests_TestData/ImageRotationTestData/image3.jpg");
    img4 = imread("Debug/posetests_TestData/ImageRotationTestData/image4.jpg");
    Q1 = imread("Debug/posetests_TestData/ImageRotationTestData/Q1.jpg");
    Q2 = imread("Debug/posetests_TestData/ImageRotationTestData/Q2.jpg");
    Q3 = imread("Debug/posetests_TestData/ImageRotationTestData/Q3.jpg");
    Q4 = imread("Debug/posetests_TestData/ImageRotationTestData/Q4.jpg");
  }
  else
  {
    img1 = imread("posetests_TestData/ImageRotationTestData/image1.jpg");
    img2 = imread("posetests_TestData/ImageRotationTestData/image2.jpg");
    img3 = imread("posetests_TestData/ImageRotationTestData/image3.jpg");
    img4 = imread("posetests_TestData/ImageRotationTestData/image4.jpg");
    Q1 = imread("posetests_TestData/ImageRotationTestData/Q1.jpg");
    Q2 = imread("posetests_TestData/ImageRotationTestData/Q2.jpg");
    Q3 = imread("posetests_TestData/ImageRotationTestData/Q3.jpg");
    Q4 = imread("posetests_TestData/ImageRotationTestData/Q4.jpg");
  }
#else
  Mat img1 = imread("Release/posetests_TestData/ImageRotationTestData/image1.jpg");
  Mat img2 = imread("Release/posetests_TestData/ImageRotationTestData/image2.jpg");
  Mat img3 = imread("Release/posetests_TestData/ImageRotationTestData/image3.jpg");
  Mat img4 = imread("Release/posetests_TestData/ImageRotationTestData/image4.jpg");
  Mat Q1 = imread("Release/posetests_TestData/ImageRotationTestData/Q1.jpg");
  Mat Q2 = imread("Release/posetests_TestData/ImageRotationTestData/Q2.jpg");
  Mat Q3 = imread("Release/posetests_TestData/ImageRotationTestData/Q3.jpg");
  Mat Q4 = imread("Release/posetests_TestData/ImageRotationTestData/Q4.jpg");
#endif  // DEBUG
#else
    Mat img1 = imread("posetests_TestData/ImageRotationTestData/image1.jpg");
    Mat img2 = imread("posetests_TestData/ImageRotationTestData/image2.jpg");
    Mat img3 = imread("posetests_TestData/ImageRotationTestData/image3.jpg");
    Mat img4 = imread("posetests_TestData/ImageRotationTestData/image4.jpg");
    Mat Q1 = imread("posetests_TestData/ImageRotationTestData/Q1.jpg");
    Mat Q2 = imread("posetests_TestData/ImageRotationTestData/Q2.jpg");
    Mat Q3 = imread("posetests_TestData/ImageRotationTestData/Q3.jpg");
    Mat Q4 = imread("posetests_TestData/ImageRotationTestData/Q4.jpg");
#endif  // WINDOWS
    imwrite("Q01.jpg", Q1);
    imwrite("Q02.jpg", Q2);
    imwrite("Q03.jpg", Q3);
    imwrite("Q04.jpg", Q4);

    // Prepare input data
    Size img_size = img1.size();
    
    float angle = 45; // the rotetion angle
    float x1 = 10.0, x2 = 39.0, y1 = 20.0, y2 = 59.0; // the rectangle vertices
    POSERECT <Point2f> rect = CreateRect(x1, x2, y1, y2);
    POSERECT <Point2f> RotatedRect = RotateRect(rect, angle);
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

    imwrite("derotated_image1.jpg", X1);
    imwrite("derotated_image2.jpg", X2);
    imwrite("derotated_image3.jpg", X3);
    imwrite("derotated_image4.jpg", X4);

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

TEST(DetectorTests, getBoneLength)
{
    const int nBins = 8;
    TestingDetector detector;
    Point2f begin(0, 0), end(1, 0);

    float length = detector.GetBoneLength(begin, end);
    cout << length;
    EXPECT_EQ(end.x - begin.x, length);
}

TEST(DetectorTests, getBoneWidth)
{
    const int nBins = 8;
    TestingDetector detector;
    BodyPart part;
    float lwRatio = 2.0;
    part.setLWRatio(lwRatio);
    float length = 1;

    float width = detector.GetBoneWidth(length, part);
    cout << length;
    EXPECT_EQ(length / lwRatio, width);
}

//Temporary test
TEST(DetectorTests, merge)
{
    Point2f center = Point2f(10, 10);
    vector<Point2f> polygon = { center, Point2f(6, 18), Point2f(14, 18), Point2f(14, 2) };
    bool isOccluded = false;
    float scoreCoeff = 1;

    // Create input vectors
    int partsCount = 16;
    vector<float> A_angles = { 1, 2, 3, 4, 5 };
    vector<float> B_angles = { 1, 1, 2, 2, 2, 3, 4, 2, 5 };
    vector<map<int, vector<Score>>> All_scores;
    All_scores.reserve(partsCount);
    vector <vector <LimbLabel>> A;
    vector <vector <LimbLabel>> B;

    for (int LimbID = 0; LimbID < partsCount; LimbID++)
    {
        vector <LimbLabel> A_temp, B_temp;
        map<int, vector<Score>> label_scores;
        for (int labelID = 0; labelID < A_angles.size(); labelID++)
        {
            int k = A_angles[labelID];
            Score score1(k / 10, "", scoreCoeff);
            Score score2(k / 10 + 2, "", scoreCoeff);
            vector <Score> scores;
            scores.push_back(score1);
            scores.push_back(score2);
            LimbLabel label(LimbID, center*k, A_angles[labelID], polygon, scores, isOccluded);
            A_temp.push_back(label);
            label_scores[k - 1].push_back(score1);
            label_scores[k - 1].push_back(score2);
            scores.clear();
        }
        for (int labelID = 0; labelID < B_angles.size(); labelID++)
        {
            int k = B_angles[labelID];
            Score score1(k / 10, "", scoreCoeff);
            Score score2(k / 10 + 2, "", scoreCoeff);
            vector <Score> scores;
            scores.push_back(score1);
            scores.push_back(score2);
            Point2f Center = center*k;
            LimbLabel label(LimbID, center*k, B_angles[labelID], polygon, scores, isOccluded);
            B_temp.push_back(label);
            label_scores[k - 1].push_back(score1);
            label_scores[k - 1].push_back(score2);
            scores.clear();
        }
        Score score(10, "", scoreCoeff);
        vector <Score> scores;
        scores.push_back(score);
        LimbLabel label(LimbID, Point2f(0,0), 2, polygon, scores, isOccluded);
        B_temp.push_back(label);
        label_scores[label_scores.size()].push_back(score);
        All_scores.push_back(label_scores);
        A.push_back(A_temp);
        B.push_back(B_temp);
        A_temp.clear();
        B_temp.clear();
        label_scores.clear();
    }

    // Create expected vector
    vector<float> All_angles = { 1, 2, 3, 4, 5 };
    vector <vector <LimbLabel>> Expected;
    for (int LimbID = 0; LimbID < partsCount; LimbID++)
    {
        vector <LimbLabel> temp;
        for (int labelID = 0; labelID < All_angles.size(); labelID++)
        {
            float k = All_angles[labelID];
            Score score1(k / 10, "", scoreCoeff);
            Score score2(k / 10 + 2, "", scoreCoeff);
            vector <Score> scores;
            scores.push_back(score1);
            scores.push_back(score2);
            LimbLabel label(LimbID, center*k, All_angles[labelID], polygon, scores, isOccluded);
            temp.push_back(label);
            scores.clear();
        }
        Score score(10, "", scoreCoeff);
        vector <Score> scores;
        scores.push_back(score);
        LimbLabel label(LimbID, Point2f(0, 0), 2, polygon, scores, isOccluded);
        temp.push_back(label);
        Expected.push_back(temp);
        temp.clear();
    }

    // Calculate actual value 
    TestingDetector detector;
    vector <vector <LimbLabel>> Actual = detector.merge(A, B);

    // Compare
    EXPECT_EQ(Expected.size(), Actual.size());
    cout << Expected.size() << endl;
    cout << Expected[0].size() << endl;
    for (int limbID = 0; limbID < partsCount; limbID++)
    {
        EXPECT_EQ(Expected[limbID].size(), Actual[limbID].size());
        for (int labelID = 0; labelID < Expected[limbID].size(); labelID++)
        {
            EXPECT_EQ(Expected[limbID][labelID].getLimbID(), Actual[limbID][labelID].getLimbID());
            EXPECT_EQ(Expected[limbID][labelID].getCenter(), Actual[limbID][labelID].getCenter());
            EXPECT_EQ(Expected[limbID][labelID].getAngle(), Actual[limbID][labelID].getAngle());
            EXPECT_EQ(All_scores[limbID][labelID], Actual[limbID][labelID].getScores());
            EXPECT_EQ(Expected[limbID][labelID].getPolygon(), Actual[limbID][labelID].getPolygon());
        }
    }
}

