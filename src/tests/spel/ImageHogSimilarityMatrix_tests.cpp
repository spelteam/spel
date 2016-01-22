// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include "imagehogsimilaritymatrix.hpp"
#include "TestsFunctions.hpp"


using namespace std;
using namespace cv;

namespace SPEL
{
  
  TEST(ImageHogSimilarityMatrix, defaultConstructor)
  {
    ImageHogSimilarityMatrix X;
    EXPECT_EQ(uint32_t(0x4948534D), X.getID());
    EXPECT_EQ(9, X.nbins);
    EXPECT_EQ(cv::Size(16, 16), X.blockSize);
    EXPECT_EQ(cv::Size(8, 8), X.blockStride);
    EXPECT_EQ(cv::Size(8, 8), X.cellSize);
    EXPECT_EQ(0.2, X.thresholdL2hys);
    EXPECT_EQ(true, X.gammaCorrection);
    EXPECT_EQ(64, X.nlevels);
    EXPECT_EQ(cv::Size(8, 8), X.wndStride);
    EXPECT_EQ(cv::Size(32, 32), X.padding);
    EXPECT_EQ(1, X.derivAperture);
    EXPECT_EQ(cv::HOGDescriptor::L2Hys, X.histogramNormType);
  }

  TEST(ImageHogSimilarityMatrix, CopyConstructor)
  {
    String FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
      if (IsDebuggerPresent())
          FilePath = "Debug/" + FilePath;
#endif

    ImageHogSimilarityMatrix X;
    bool b = X.read(FilePath + "In_Matrix.txt");
    ASSERT_TRUE(b);

    ImageHogSimilarityMatrix Y(X);

    ASSERT_EQ(X.size(), Y.size());
    for (int i = 0; i < X.size(); i++)
      for (int k = 0; k < X.size(); k++)
      {
        EXPECT_EQ(X.at(i, k), Y.at(i, k));
        EXPECT_EQ(X.getShift(i, k), Y.getShift(i, k));
      }
  }  

  TEST(ImageHogSimilarityMatrix, Constructor)
  {
    vector<Frame*> frames = LoadTestProject("speltests_TestData/SurfDetectorTestsData/", "trijumpSD_shortcut.xml");
    ASSERT_GT(frames.size(), 0);

    ImageHogSimilarityMatrix X(frames);
    X.write("HogSimilarityMatrix.txt");

  }

  TEST(ImageHogSimilarityMatrix, MoveConstructor)
  {
    //Prepare test data
    String FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif

    ImageHogSimilarityMatrix X;
    bool b = X.read(FilePath + "In_Matrix.txt");
    ASSERT_TRUE(b);

    //Create actual value
    ImageHogSimilarityMatrix Y(static_cast<ImageHogSimilarityMatrix&&>(X));
    Mat X_ShiftMatrix = X.imageShiftMatrix;
    Mat Y_ShiftMatrix = Y.imageShiftMatrix;
    int X_rows = X_ShiftMatrix.size().height;
    int Y_rows = Y_ShiftMatrix.size().height;

    //Compare
    ASSERT_EQ(X_rows, Y_rows);
    for (int i = 0; i < X_rows; i++)
      for (int k = 0; k < Y_rows; k++)
      {
        EXPECT_EQ(X.at(i, k), Y.at(i, k));
        EXPECT_EQ(X_ShiftMatrix.at<Point2f>(i, k), Y_ShiftMatrix.at<Point2f>(i, k));
      }

    X_ShiftMatrix.release();
    Y_ShiftMatrix.release();
  }


  TEST(ImageHogSimilarityMatrix, Operators)
  {
    //Prepare test data
    String FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif

    bool b;
    ImageHogSimilarityMatrix X, Y;
    b = X.read(FilePath + "In_Matrix.txt");
    ASSERT_TRUE(b);

    //ImagMaskSimilarityMatrix operator "="
    Y = X;
    EXPECT_TRUE(Y == X);
  }

  TEST(ImageHogSimilarityMatrix, calculateROI)
  {
    //Prepare test data
    int rows = 100, cols = 200;
    Point2i A(20, 40), B(60, 20);
    vector<Point2f> rect = { Point2i(A.x, A.y), Point2i(B.x, A.y), Point2i(B.x, B.y), Point2i(A.x, B.y) };

    Mat Mask = Mat(rows, cols, CV_8UC1, 0);
    Mat Image = Mat(rows, cols, CV_8UC3, Scalar(0, 0, 0));
    for (int x = A.x; x != B.x; x++)
      for (int y = B.y; y != A.y; y++)
        Image.at<Vec3b>(y, x) = Vec3b(255, 255, 255);

    cvtColor(Image, Mask, CV_BGR2GRAY);
    imwrite("ROI_Mask.jpg", Mask);

    //Create actual value
    Frame* frame = new Lockframe();
    frame->setMask(Mask);
    Point2i TopLeft, BottomRight;
    ImageHogSimilarityMatrix X;
    X.calculateROI(frame, TopLeft, BottomRight);

    //Compare
    EXPECT_EQ(A, TopLeft);
    EXPECT_EQ(B, BottomRight);
    Image.release();
    Mask.release();
  }

}