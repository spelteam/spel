// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include <imagesimilaritymatrix.hpp>
#include <imagepixelsimilaritymatrix.hpp>
#include "TestsFunctions.hpp"


using namespace std;
using namespace cv;

namespace SPEL
{
  class TestsMatrix : public ImagePixelSimilarityMatrix
  {
  public:
    Mat getImageShiftMatrix();
    float ISMCell(Frame* left, Frame* right, int maxFrameHeight);
  };

  Mat TestsMatrix::getImageShiftMatrix()
  {
    return imageShiftMatrix;
  }

  float TestsMatrix::ISMCell(Frame* left, Frame* right, int maxFrameHeight)
  {
    computeISMcell(left, right, maxFrameHeight);
    return imageSimilarityMatrix.at<float>(left->getID(), right->getID());
  }

  class ImageSimilarityMatrixTests : public testing::Test
  {
  protected:
    virtual void SetUp()
    {
      FilePath = "speltests_TestData/SimilarityMatrixTestsData/";
#if defined(WINDOWS) && defined(_MSC_VER)
      if (IsDebuggerPresent())
          FilePath = "Debug/" + FilePath;
#endif
      a = A.read(FilePath + "In_Matrix.txt");
    }

    String FilePath;
    ImagePixelSimilarityMatrix A;
    bool a;
  };

  TEST_F(ImageSimilarityMatrixTests, WriteAndRead)
    {
      int rows = 3, cols = rows;
      TestsMatrix X, Y;

      // Testing "read""
      bool b = X.read(FilePath + "In_Matrix.txt");
      Mat X_ShiftMatrix = X.getImageShiftMatrix();

      ASSERT_TRUE(b);
      ASSERT_EQ(rows, X.size());
      for (int i = 0; i < rows; i++)
        for (int k = 0; k < cols; k++)
        {
          float x = float(rows*i + k +1);
          EXPECT_EQ(x, X.at(i,k));
          EXPECT_EQ(Point2f(x/10, x/10), X_ShiftMatrix.at<Point2f>(i, k));
        }
      
      // Testing "write"
      b = X.write(FilePath + "Out_Matrix.txt");
      ASSERT_TRUE(b);

      b = Y.read(FilePath + "Out_Matrix.txt");
      Mat Y_ShiftMatrix = Y.getImageShiftMatrix();
      ASSERT_TRUE(b);
      ASSERT_EQ(rows, Y.size());
      for (int i = 0; i < rows; i++)
        for (int k = 0; k < cols; k++)
        {
          float x = float(rows*i + k + 1);
          EXPECT_EQ(x, Y.at(i, k));
          EXPECT_EQ(Point2f(x / 10, x / 10), Y_ShiftMatrix.at<Point2f>(i, k));
        }  	
      X_ShiftMatrix.release();
    }

  TEST_F(ImageSimilarityMatrixTests, at)
  {
    EXPECT_EQ(1.0, A.at(0, 0));
  }

  TEST_F(ImageSimilarityMatrixTests, getShift)
  {
    EXPECT_EQ(Point2f(0.1f, 0.1f), A.getShift(0, 0));
  }

  TEST_F(ImageSimilarityMatrixTests, Operators)
  {
    ImagePixelSimilarityMatrix B, C, D;
    bool b, c;

    b = B.read(FilePath + "In_Matrix.txt");
    c = C.read(FilePath + "In_Matrix2.txt");
    ASSERT_TRUE(a && b && c);

    // Operator "=="
    EXPECT_TRUE(A == B); // This does not work. Error in "ImageSimilarityMatrix::operator=="???
    EXPECT_FALSE(A == C);

    //Operator "!="
    EXPECT_FALSE(A != B);
    EXPECT_TRUE(A != C);

    //Operator "="
    C = A;
    EXPECT_TRUE(A == C);

    //Move assigment operator
    D = static_cast<ImagePixelSimilarityMatrix&&>(C);
    EXPECT_TRUE(D == C);
  }

  TEST_F(ImageSimilarityMatrixTests, min)
   {
     ASSERT_TRUE(a);
     EXPECT_FLOAT_EQ(2.0, A.min());
   }

  TEST_F(ImageSimilarityMatrixTests, max)
  {
    ASSERT_TRUE(a);
    EXPECT_FLOAT_EQ(9.0, A.max());
  }

  TEST_F(ImageSimilarityMatrixTests, mean)
  {
    ASSERT_TRUE(a);
    EXPECT_FLOAT_EQ(5.0, A.mean());
  }

  TEST_F(ImageSimilarityMatrixTests, stddev)
  {
    ASSERT_TRUE(a);
    EXPECT_FLOAT_EQ(2.1602468f, A.stddev());
  }

  TEST_F(ImageSimilarityMatrixTests, getPathCost)
  {
    vector<int> path = { 0, 1, 2 };

    ASSERT_TRUE(a);
    EXPECT_FLOAT_EQ(8.0, A.getPathCost(path));

    path.push_back(4);
    EXPECT_THROW(A.getPathCost(path), std::out_of_range);
    path.clear();
  }

  TEST_F(ImageSimilarityMatrixTests, size)
  {
    ASSERT_TRUE(a);
    EXPECT_EQ(3, A.size());
  }

  TEST_F(ImageSimilarityMatrixTests, clone)
  {
    Mat B = A.clone();
    ASSERT_EQ((int)A.size(), B.size().width);
    for (unsigned int i = 0; i < A.size(); i++)
      for (unsigned int k = 0; k < A.size(); k++)
      {
        float x = float(A.size()*i + k + 1);
        EXPECT_EQ(x, B.at<float>(i, k));
      }
  }

  TEST_F(ImageSimilarityMatrixTests, DefaultConstructor)
  {
    TestsMatrix X;
    Mat X_ShiftMatrix = X.getImageShiftMatrix();

    EXPECT_EQ(0, X.size());
    EXPECT_EQ(Size(0,0), X_ShiftMatrix.size());
  }

  TEST_F(ImageSimilarityMatrixTests, CopyConstructor)
  {
    TestsMatrix X;
    bool b = X.read(FilePath + "In_Matrix.txt");

    ASSERT_TRUE(b);
    TestsMatrix Y(X);
    Mat X_ShiftMatrix = X.getImageShiftMatrix();
    Mat Y_ShiftMatrix = Y.getImageShiftMatrix();
    int X_rows = X_ShiftMatrix.size().height;
    int Y_rows = Y_ShiftMatrix.size().height;

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

  TEST_F(ImageSimilarityMatrixTests, MoveConstructor)
  {
      TestsMatrix X;
      bool b = X.read(FilePath + "In_Matrix.txt");

      ASSERT_TRUE(b);
      TestsMatrix Y(static_cast<TestsMatrix&&>(X));
      Mat X_ShiftMatrix = X.getImageShiftMatrix();
      Mat Y_ShiftMatrix = Y.getImageShiftMatrix();
      int X_rows = X_ShiftMatrix.size().height;
      int Y_rows = Y_ShiftMatrix.size().height;

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

  TEST(ImageSimilarityMatrixTests_, computeISMcell)
  {
    //Load the input data
    float r = 0.5 * 135;
    vector<Frame*> frames = LoadTestProject("speltests_TestData/SimilarityMatrixTestsData/", "Abstraction.xml");
    cout << "frames.size = " << frames.size() << endl;
    cout << "Image.size = " << frames[0]->getImage().size() << endl;
    cout << "Mask.size = " << frames[0]->getMask().size() << endl;

    //Create expected value
    ImagePixelSimilarityMatrix ISM_expected(frames);
    const float A = 3*255 * 255 * pow(r, 2)*(4 - 3.14f);
    ISM_expected.imageSimilarityMatrix.at<float>(0, 1) = A;
    ISM_expected.imageSimilarityMatrix.at<float>(1, 4) = A;
    ISM_expected.imageSimilarityMatrix.at<float>(1, 5) = A;
    ISM_expected.imageSimilarityMatrix.at<float>(2, 3) = A;

    const float B = A + pow(r,2)*255*255;
    ISM_expected.imageSimilarityMatrix.at<float>(0, 3) = B;
    ISM_expected.imageSimilarityMatrix.at<float>(1, 2) = B;
    ISM_expected.imageSimilarityMatrix.at<float>(3, 4) = B;
    ISM_expected.imageSimilarityMatrix.at<float>(3, 5) = B;

    const float C = 4 * pow(r, 2)*255*255;
    ISM_expected.imageSimilarityMatrix.at<float>(0, 2) = C;
    ISM_expected.imageSimilarityMatrix.at<float>(2, 4) = C;
    ISM_expected.imageSimilarityMatrix.at<float>(2, 5) = C;

    const float D = 3.14f*pow(r, 2)*255.0f*255.0f;
    ISM_expected.imageSimilarityMatrix.at<float>(1, 3) = D;

    ISM_expected.imageSimilarityMatrix.at<float>(0, 4) = 0;
    ISM_expected.imageSimilarityMatrix.at<float>(0, 5) = 0;
    ISM_expected.imageSimilarityMatrix.at<float>(4, 5) = 0;
    for (unsigned int i = 0; i < frames.size(); i++)
      ISM_expected.imageSimilarityMatrix.at<float>(i, i) = 0;

    for (unsigned int i = 1; i < frames.size(); i++)
      for (unsigned int k = 0; k <= i; k++)
        ISM_expected.imageSimilarityMatrix.at<float>(i, k) = ISM_expected.imageSimilarityMatrix.at<float>(k, i);

    ISM_expected.write("seq_ISM_expected.txt");

    //Create actual value
    ImagePixelSimilarityMatrix ISM_actual(frames);

    //Put results
    for (unsigned int i = 0; i < frames.size(); i++)
      for (unsigned int k = 0; k <= i; k++)
      {
        ISM_actual.ImagePixelSimilarityMatrix::computeISMcell(frames[i], frames[k], 0);
        cout << "ISM[" << i << ", " << k << "] = ";
        cout << ISM_actual.at(frames[i]->getID(), frames[k]->getID()) << " ~ ";
        cout << ISM_expected.at(frames[i]->getID(), frames[k]->getID()) << endl;	 
      }

    //Compare
    float error = 0.1e+5;
    for (unsigned int i = 0; i < frames.size(); i++)
      for (unsigned int k = 0; k < frames.size(); k++)
      {
        //ISM_actual.computeISMcell(frames[i], frames[k], 0);
        EXPECT_NEAR(ISM_expected.at(frames[i]->getID(), frames[k]->getID()), ISM_actual.at(frames[i]->getID(), frames[k]->getID()), error);
      }	
    ISM_expected.write("seq_ISM_actual.txt");

    cout << "======================================================" << endl;
    cout << "Testing function 'buildImageSimilarityMatrix:'" << endl;
    //Testing function "buildImageSimilarityMatrix"
    for (unsigned int i = 0; i < frames.size(); i++)
      for (unsigned int k = 0; k < frames.size(); k++)
        ISM_actual.imageSimilarityMatrix.at<float>(i, k) = 0;
    ISM_actual.buildImageSimilarityMatrix(frames, 0);
    for (unsigned int i = 0; i < frames.size(); i++)
      for (unsigned int k = 0; k < frames.size(); k++)
        EXPECT_NEAR(ISM_expected.at(frames[i]->getID(), frames[k]->getID()), ISM_actual.at(frames[i]->getID(), frames[k]->getID()), error);
  }
  /*
  TEST(ImageSimilarityMatrixTests_, computeMSMcell)
  {
    //Load the input data
    //float r = 0.5 * 135;
    vector<Frame*> frames = LoadTestProject("speltests_TestData/SimilarityMatrixTestsData/", "Abstraction.xml");
    cout << "frames.size = " << frames.size() << endl;
    cout << "Image.size = " << frames[0]->getImage().size() << endl;
    cout << "Mask.size = " << frames[0]->getMask().size() << endl;

    //Create expected value
    ImagePixelSimilarityMatrix ISM_expected(frames);
    const float A = 3.14f/4;
    ISM_expected.imageSimilarityMatrix.at<float>(0, 1) = A;
    ISM_expected.imageSimilarityMatrix.at<float>(1, 4) = A;
    ISM_expected.imageSimilarityMatrix.at<float>(1, 5) = A;
    ISM_expected.imageSimilarityMatrix.at<float>(2, 3) = A;

    const float B = A; //+ pow(r,2)*255*255;
    ISM_expected.imageSimilarityMatrix.at<float>(0, 3) = B;
    ISM_expected.imageSimilarityMatrix.at<float>(1, 2) = B;
    ISM_expected.imageSimilarityMatrix.at<float>(3, 4) = B;
    ISM_expected.imageSimilarityMatrix.at<float>(3, 5) = B;

    const float C = 1; //4 * pow(r, 2) * 255 * 255;
    ISM_expected.imageSimilarityMatrix.at<float>(0, 2) = C;
    ISM_expected.imageSimilarityMatrix.at<float>(2, 4) = C;
    ISM_expected.imageSimilarityMatrix.at<float>(2, 5) = C;

    const float D = 1; //3.14*pow(r, 2) * 255 * 255;
    ISM_expected.imageSimilarityMatrix.at<float>(1, 3) = D;

    ISM_expected.imageSimilarityMatrix.at<float>(0, 4) = 0;
    ISM_expected.imageSimilarityMatrix.at<float>(0, 5) = 0;
    ISM_expected.imageSimilarityMatrix.at<float>(4, 5) = 0;
    for (unsigned int i = 0; i < frames.size(); i++)
      ISM_expected.imageSimilarityMatrix.at<float>(i, i) = 0;

    for (unsigned int i = 1; i < frames.size(); i++)
      for (unsigned int k = 0; k <= i; k++)
        ISM_expected.imageSimilarityMatrix.at<float>(i, k) = ISM_expected.imageSimilarityMatrix.at<float>(k, i);

    ISM_expected.write("seq_MSM_expected.txt");

    //Craate actual value
    ImagePixelSimilarityMatrix ISM_actual(frames);

    //Put results
    for (unsigned int i = 0; i < frames.size(); i++)
      for (unsigned int k = 0; k <= i; k++)
      {
        ISM_actual.computeISMcell(frames[i], frames[k], 0);
        cout << "MSM[" << i << ", " << k << "] = ";
        cout << ISM_expected.at(frames[i]->getID(), frames[k]->getID()) << " ~ ";
        cout << ISM_actual.at(frames[i]->getID(), frames[k]->getID()) << endl;
      }

    //Compare
    float error = 0.1e+5;
    for (unsigned int i = 0; i < frames.size(); i++)
      for (unsigned int k = 0; k < frames.size(); k++)
      {
        ISM_actual.computeISMcell(frames[i], frames[k], 0);
        EXPECT_NEAR(ISM_expected.at(frames[i]->getID(), frames[k]->getID()), ISM_actual.at(frames[i]->getID(), frames[k]->getID()), error);
      }	
    ISM_expected.write("seq_MSM_actual.txt");
    
    //Testing function "buildMaskSimilarityMatrix"
    for (unsigned int i = 0; i < frames.size(); i++)
      for (unsigned int k = 0; k < frames.size(); k++)
        ISM_actual.imageSimilarityMatrix.at<float>(i,k) = 0;
    ISM_actual.buildMaskSimilarityMatrix(frames, 0);
    for (unsigned int i = 0; i < frames.size(); i++)
      for (unsigned int k = 0; k < frames.size(); k++)
        EXPECT_NEAR(ISM_expected.at(frames[i]->getID(), frames[k]->getID()), ISM_actual.at(frames[i]->getID(), frames[k]->getID()), error);
    
  }
  */
  TEST(ImageSimilarityMatrixTests_, setID_getID)
  {
    int id = 356;
    ImagePixelSimilarityMatrix C;
    C.setID(id);
    EXPECT_EQ(id, C.getID());
  }

}
