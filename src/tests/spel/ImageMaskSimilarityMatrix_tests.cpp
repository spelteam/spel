// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include <imagesimilaritymatrix.hpp>
#include <imagemasksimilaritymatrix.hpp>
#include "TestsFunctions.hpp"


using namespace std;
using namespace cv;

namespace SPEL
{

  TEST(MaskSimilarityMatrixTests, DefaultConstructor)
  {
    //Prepare test data
    String FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif

    //Create actual value
    ImageMaskSimilarityMatrix X;
    Size ShiftMatrix_Size = X.imageShiftMatrix.size();

    //Compare
    EXPECT_EQ(0, X.size());
    EXPECT_EQ(Size(0,0), ShiftMatrix_Size);
  }

  TEST(MaskSimilarityMatrixTests, CopyConstructor)
  {
    //Prepare test data
    String FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif


    ImageMaskSimilarityMatrix X;
    bool b = X.read(FilePath + "In_Matrix.txt");
    ASSERT_TRUE(b);

    //Create actual value
    ImageMaskSimilarityMatrix Y(X);
    Mat X_ShiftMatrix = X.imageShiftMatrix;
    Mat Y_ShiftMatrix = Y.imageShiftMatrix;
    int X_rows = X_ShiftMatrix.size().height;
    int Y_rows = Y_ShiftMatrix.size().height;

    //Compare
    ASSERT_EQ(X_rows, Y_rows);
    for (int i = 0; i < X_rows; i++)
      for (int k = 0; k < Y_rows; k++)
      {
        EXPECT_EQ(X.at(i, k), Y.at(i,k));
        EXPECT_EQ(X_ShiftMatrix.at<Point2f>(i, k), Y_ShiftMatrix.at<Point2f>(i, k));
      }

    X_ShiftMatrix.release();
    Y_ShiftMatrix.release();
  }

  TEST(MaskSimilarityMatrixTests, MoveConstructor)
  {
    //Prepare test data
    String FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif

    ImageMaskSimilarityMatrix X;
    bool b = X.read(FilePath + "In_Matrix.txt");
    ASSERT_TRUE(b);

    //Create actual value
    ImageMaskSimilarityMatrix Y(static_cast<ImageMaskSimilarityMatrix&&>(X));
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

  TEST(MaskSimilarityMatrixTests, Operators)
  {
    //Prepare test data
    String FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
      FilePath = "Debug/" + FilePath;
#endif

    bool b;
    ImageMaskSimilarityMatrix X, Y;
    b = X.read(FilePath + "In_Matrix.txt");
    ASSERT_TRUE(b);

    //ImagMaskSimilarityMatrix operator "="
    Y = X;
    EXPECT_TRUE(Y == X);
  }

  TEST(MaskSimilarityMatrixTests, computeISMCell)
  {
    vector<Frame*> frames = LoadTestProject("speltests_TestData/SimilarityMatrixTestsData/", "Abstraction.xml");
    ASSERT_TRUE(frames.size() > 0);

    ImageMaskSimilarityMatrix X;
    X.imageSimilarityMatrix = Mat(frames.size(), frames.size(), cv::DataType<float>::type); 
    X.computeISMcell(frames[0], frames[1], 0);
    cout << X.at(0, 1) << endl;

    //EXPECT_EQ(static_cast<float>(2.0/M_PI), X.at(0, 1));
    EXPECT_NEAR(static_cast<float>(2.0/M_PI), X.at(0, 1), 0.01f);
  }

  TEST(MaskSimilarityMatrixTests, buildISM)
  {
    vector<Frame*> frames = LoadTestProject("speltests_TestData/SimilarityMatrixTestsData/", "Abstraction.xml");
    int n = frames.size();
    ASSERT_TRUE(frames.size() > 0);

    ImageMaskSimilarityMatrix X(frames);
    ASSERT_EQ(n, X.size());

    float x = static_cast<float>(2.0 / M_PI);
    for (int i = 0; i < n; i++)
      for (int k = 0; k < n; k++)
      {
        if ((i != k) && ((k + i*(n-1)) % 2 > 0) )
          EXPECT_NEAR(x, X.at(i, k), 0.01f);
        else 
          EXPECT_EQ(0.0f, X.at(i, k));
      }
  }

}