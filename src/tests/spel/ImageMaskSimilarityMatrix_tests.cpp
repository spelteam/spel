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

    ImageMaskSimilarityMatrix Temp(X);
    Mat X_ShiftMatrix;
    X_ShiftMatrix = X.imageShiftMatrix.clone();
    int X_rows = X_ShiftMatrix.size().height;
    
    //Create actual value
    ImageMaskSimilarityMatrix Y(static_cast<ImageMaskSimilarityMatrix&&>(X));
    Mat Y_ShiftMatrix = Y.imageShiftMatrix;
    int Y_rows = Y_ShiftMatrix.size().height;

    //Compare
    ASSERT_EQ(X_rows, Y_rows);
    for (int i = 0; i < X_rows; i++)
      for (int k = 0; k < Y_rows; k++)
      {
        EXPECT_EQ(Temp.at(i, k), Y.at(i, k));
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
    TestProjectLoader project("speltests_TestData/SimilarityMatrixTestsData/", "Abstraction.xml");
    vector<Frame*> frames = project.getFrames();

    ASSERT_TRUE(frames.size() > 0);

    ImageMaskSimilarityMatrix X;

    X.imageSimilarityMatrix = Mat(frames.size(), frames.size(), cv::DataType<float>::type, 0.0f);
    X.imageShiftMatrix = Mat(frames.size(), frames.size(), cv::DataType<Point2f>::type);

    X.computeISMcell(frames[3], frames[2], 0);

    float error = 0.076f;
    EXPECT_NEAR(static_cast<float>(M_PI/4.0f), X.at(3, 2), error);
    EXPECT_NEAR(static_cast<float>(M_PI/4.0f), X.at(2, 3), error);

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    frames.clear();
  }

  TEST(MaskSimilarityMatrixTests, buildISM)
  {
    TestProjectLoader project("speltests_TestData/SimilarityMatrixTestsData/", "Abstraction.xml");
    vector<Frame*> frames = project.getFrames();

    int n = frames.size();
    ASSERT_TRUE(frames.size() > 0);

    frames[n - 1]->setImage(frames[1]->getImage());
    frames[n - 1]->setMask(frames[1]->getMask());

    ImageMaskSimilarityMatrix X(frames);
    ASSERT_EQ(n, X.size());

    X.write("MSM.txt");

    float error = 0.06f;
    float x = static_cast<float>(M_PI/4.0);
    for (int i = 0; i < n; i++)
      for (int k = 0; k < n; k++)
        if (i != k)
        {
          if ((k + i*(n-1)) % 2 > 0 )
            EXPECT_NEAR(x, X.at(i, k), error) << "i = " << i << ", k= " << k << endl;        
          else
            EXPECT_NEAR(1.0f, X.at(i, k), error) << "i = " << i << ", k= " << k << endl;
        }
    for (int i = 0; i < n; i++)
      EXPECT_EQ(0.0f, X.at(i, i)) << "i = " << i << endl;

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    frames.clear();
  }

  // Write alternative MSM
  TEST(MaskSimilarityMatrixTests, WriteNewMSM)
  {
    TestProjectLoader project("speltests_TestData/testdata1/", "trijumpSD_new.xml");
    //project.Load("speltests_TestData/SimilarityMatrixTestsData/", "Abstraction.xml");
    vector<Frame*> frames = project.getFrames();

    //vector<Frame*> frames = LoadTestProject("speltests_TestData/SimilarityMatrixTestsData/", "Abstraction.xml");
    ASSERT_TRUE(frames.size() > 0);

    ImageMaskSimilarityMatrix MSM;
    MSM.buildImageSimilarityMatrix(frames, 0, 0);
    /*
    for (int i = 0; i < frames.size(); i++)
    {
      for (unsigned int k = 0; k < frames.size(); k++)
      {
        cout << MSM.at(i, k);
        if(k != (frames.size() - 1)) cout << "\t";
      }
      cout << endl;
    }*/
    MSM.write("AlternativeMSM.txt");

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    frames.clear();
  }

  // Testing alternative buildMSM function
  TEST(MaskSimilarityMatrixTests, buildNewMSM)
  {
    TestProjectLoader project("speltests_TestData/SimilarityMatrixTestsData/", "Abstraction.xml");
    vector<Frame*> frames = project.getFrames();

    int n = frames.size();
    ASSERT_TRUE(frames.size() > 0);

    frames[n - 1]->setImage(frames[1]->getImage());
    frames[n - 1]->setMask(frames[1]->getMask());

    ImageMaskSimilarityMatrix X(frames, 0, 0);
    ASSERT_EQ(n, X.size());

    X.write("MSM.txt");

    float error = 0.06f;
    float x = static_cast<float>(M_PI/4.0);
    for (int i = 0; i < n; i++)
      for (int k = 0; k < n; k++)
        if (i != k)
        {
          if ((k + i*(n-1)) % 2 > 0 )
            EXPECT_NEAR(x, X.at(i, k), error) << "i = " << i << ", k= " << k << endl;        
          else
            EXPECT_NEAR(1.0f, X.at(i, k), error) << "i = " << i << ", k= " << k << endl;
        }
    for (int i = 0; i < n; i++)
      EXPECT_EQ(0.0f, X.at(i, i)) << "i = " << i << endl;

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    frames.clear();
  }

}