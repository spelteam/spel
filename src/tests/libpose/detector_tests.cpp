#include <gtest/gtest.h>
#include <detector.hpp>
#include <colorHistDetector.hpp>

using namespace cv;

class TestingDetector : public ColorHistDetector
{
  public:
  Mat DeRotate(Mat imgSource, POSERECT <Point2f> &initialRect, float angle, Size size);
};

Mat TestingDetector::DeRotate(Mat imgSource, POSERECT <Point2f> &initialRect, float angle, Size size)
{
    return  TestingDetector::rotateImageToDefault(imgSource, initialRect, angle, size);
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
    for (int i = xmin; i <= xmax; i++)
        for (int j = ymin; j <= ymax; j++)
            Img.at<Vec3b>(j, i) = colour;
}

void FillRectRand(Mat &Img, POSERECT<Point2f> &rect)
{
    float xmax, ymax, xmin, ymin;
    rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
    Vec3b colour;
    const int c = 250;
    for (int i = xmin; i <= xmax; i++)
        for (int j = ymin; j <= ymax; j++)
        {
            colour = Vec3b(rand() * c / RAND_MAX, rand() * c / RAND_MAX, rand() * c / RAND_MAX);
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
    for (int i = xmin; i <= xmax; i++)
    for (int j = ymin; j <= ymax; j++)
    {
        E = Point2i(i, j);
        E = PoseHelper::rotatePoint2D(E, center, angle);
        if ((0 <  E.x < size.width) && (0 < E.y < size.height))
            img2.at<Vec3b>((int32_t)(E.y), (int32_t)(E.x)) = Img.at<Vec3b>(j, i);
    }
    return img2;
}

// Searching the rectangle extreme coordinates at image
void GetExtremePoints(Mat Img, Vec3b bgColour, Point2f &Min, Point2f &Max)
{
    Size size = Img.size();
    Min = Point2f(size.width + 1, size.height + 1);
    Max = Point2f(-1, -1);
    for (int i = 0; i < size.width; i++)
        for (int j = 0; j < size.height; j++)
            if (Img.at<Vec3b>(j, i) != bgColour)
            {
                if (i < Min.x) { Min.x = i; }
                if (i > Max.x) { Max.x = i; }
                if (j < Min.y) { Min.y = j; }
                if (j > Max.y) { Max.y = j; }
            }
}

TEST(DetectorTests, rotateImageToDefault_OneColor)
{
    // Prepare input data
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

    Vec3b colour(0, 0, 255); // color fill 

    FillRect(img1, rect, colour);
    img2 = RotateImage(img1, rect, angle);

    TestingDetector chd;
    Size size(xmax - xmin + 1, ymax - ymin + 1);

    // Testing
    Mat  X = chd.DeRotate(img2, RotatedRect, angle, size);
   
    imwrite("image.jpg", img1);
    imwrite("rotated_image.jpg", img2);
    imwrite("derotated_image.jpg", X);

    Point2f Min, Max;
 
    GetExtremePoints(X, Vec3b(255, 255, 255), Min, Max);

    // Checking extreme points
    float delta = 1; // tolerable linear error
    EXPECT_LE(Min.x, delta);
    EXPECT_LE(abs(Max.x + xmin - MaxR.x), delta);
    EXPECT_LE(Min.y, delta);
    EXPECT_LE(abs(Max.y + ymin - MaxR.y), delta);

    // Checking matching of the images points with one fill color 
    uint32_t S = 0;
    uint32_t S0 = (xmax - xmin + 1)*(ymax - ymin + 1);
    for (int i = xmin; i <= xmax; i++)
    for (int j = ymin; j <= ymax; j++)
        if (img1.at<Vec3b>(j, i) == X.at<Vec3b>(j - ymin, i - xmin))
            S++;


    float epsilon = 40; // tolerable error of the matching points number, %
    float FalsePixels = 100 * (S0 - S) / S0;
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
    Size size(xmax - xmin + 1, ymax - ymin + 1);

    Mat  X = chd.DeRotate(img2, RotatedRect, angle, size);

    imwrite("image_RandColor.jpg", img1);
    imwrite("rotated_image_RandColor.jpg", img2);
    imwrite("derotated_image_RandColor.jpg", X);

    Point2f Min, Max;

    GetExtremePoints(X, Vec3b(255, 255, 255), Min, Max);

    // Checking extreme points
    float delta = 1; // tolerable linear error
    EXPECT_LE(Min.x, delta);
    EXPECT_LE(abs(Max.x + xmin - MaxR.x), delta);
    EXPECT_LE(Min.y, delta);
    EXPECT_LE(abs(Max.y + ymin - MaxR.y), delta);

    // Checking matching of the images points with one fill color 
    uint32_t S = 0;
    uint32_t S0 = (xmax - xmin + 1)*(ymax - ymin + 1);
    for (int i = xmin; i <= xmax; i++)
    for (int j = ymin; j <= ymax; j++)
    if (img1.at<Vec3b>(j, i) == X.at<Vec3b>(j - ymin, i - xmin))
        S++;

    float epsilon = 67; // tolerable error of the matching points number, %
    float FalsePixels = 100 * (S0 - S) / S0;
    EXPECT_LE(FalsePixels, epsilon);

    img1.release();
    img2.release();
    X.release();
}
