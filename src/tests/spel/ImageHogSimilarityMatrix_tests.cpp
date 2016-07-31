// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#include <gtest/gtest.h>
#include "imagehogsimilaritymatrix.hpp"
#include "lockframe.hpp"
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
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/A/", "trijumpSD_shortcut.xml");
    vector<Frame*> frames = project.getFrames();
    ASSERT_GT(frames.size(), 0);

    ImageHogSimilarityMatrix X(frames);
    X.write("HogSimilarityMatrix.txt");

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    frames.clear();
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

    ImageHogSimilarityMatrix Temp(X);
    Mat X_ShiftMatrix;
    X_ShiftMatrix = X.imageShiftMatrix.clone();
    int X_rows = X_ShiftMatrix.size().height;

    //Create actual value
    ImageHogSimilarityMatrix Y(static_cast<ImageHogSimilarityMatrix&&>(X));
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
    Point2i A(20, 20), B(60, 40);
    vector<Point2f> rect = { Point2i(A.x, A.y), Point2i(B.x, A.y), Point2i(B.x, B.y), Point2i(A.x, B.y) };

    Mat Mask = Mat::zeros(rows, cols, CV_8UC1);
    Mat Image = Mat::zeros(rows, cols, CV_8UC3);
    for (int x = A.x; x <= B.x; x++)
      for (int y = A.y; y <= B.y; y++)
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

    //Clear
    delete frame;
  }

  TEST(ImageHogSimilarityMatrix, calculateISMCell)
  {
    //Prepare test data
    int rows = 64, cols = 128;
    Mat Image0(rows, cols, CV_8UC3, Scalar(0, 0, 0));
    Mat Image1(rows, cols, CV_8UC3, Scalar(0, 0, 0));

    Image0.at<Vec3b>(0, 0) = Vec3b(255, 255, 255);
    Image1.at<Vec3b>(0, 0) = Vec3b(255, 255, 255);
    Image0.at<Vec3b>(rows - 1, cols - 1) = Vec3b(255, 255, 255);
    Image1.at<Vec3b>(rows - 1, cols - 1) = Vec3b(255, 255, 255);

    cv::ellipse(Image0, Point(0.5f*cols, 0.5f*rows), Size(0.375f*cols, 0.375f*rows), 0.0, 0.0, 360.0, Scalar(255, 255, 255), 1, 0, 0);
    cv::ellipse(Image1, Point(0.5f*cols, 0.5f*rows), Size(0.375f*cols, 0.375f*rows), 0.0, 180.0, 360.0, Scalar(255, 255, 255), 1, 0, 0);
    //Image0.at<uchar>(0, 0) = 255;
    //Image1.at<uchar>(rows - 1, cols - 1) = 255;
      
    Mat Mask0, Mask1;

    cvtColor(Image0, Mask0, CV_RGB2GRAY);
    cvtColor(Image1, Mask1, CV_RGB2GRAY);
    imwrite("hogISM_Image0.jpg", Image0);
    imwrite("hogISM_Image1.jpg", Image1);

    vector<Frame*> frames;
    for (int i = 0; i < 2; i++)
    {
      frames.push_back(new Lockframe());
      frames[i]->setID(i);
    }
    frames[0]->setImage(Image0);
    frames[0]->setMask(Image0);
    frames[1]->setImage(Image1);
    frames[1]->setMask(Image1);

    //Create expected value
    Size winSize(128, 64); Size(0,1);
    Size blockSize(16, 16);
    Size blockStride(8, 8);
    Size cellSize(8, 8);
    int nBins = 9;
    int derivAper = 0;
    double winSigma = -1.0;	
    int histogramNormType = 0;
    double L2HysThresh = 0.2;
    bool gammaCorrection = true;
    int nLevels = 64;
    vector<float> descriptors;	  
    vector<vector<vector<float>>> GradientStrengths0, GradientStrengths1;

    HOGDescriptor d(winSize, blockSize, blockStride, cellSize, nBins, derivAper, winSigma, histogramNormType, L2HysThresh, gammaCorrection, nLevels);
    d.compute(Image0, descriptors);
    GradientStrengths0 = averageGradientStrengths(Image0, descriptors, winSize, blockSize, blockStride, cellSize, nBins, derivAper, winSigma, histogramNormType, L2HysThresh, gammaCorrection, nLevels);
    descriptors.clear();
    d.compute(Image1, descriptors);
    GradientStrengths1 = averageGradientStrengths(Image1, descriptors, winSize, blockSize, blockStride, cellSize, nBins, derivAper, winSigma, histogramNormType, L2HysThresh, gammaCorrection, nLevels);
 
    float score_expected = 0.0f;
    int N = static_cast<int>(winSize.height / cellSize.height);  
    int M = static_cast<int>(winSize.width / cellSize.width);
    for(int y = 0; y < N; y++)
      for(int x = 0; x < M; x++)
        for(int n= 0; n < nBins; n++)
          score_expected += pow(GradientStrengths1[y][x][n] - GradientStrengths0[y][x][n], 2);

    //Create actual value
    ImageHogSimilarityMatrix X(frames);
    X.computeISMcell(frames[0], frames[1], Image0.cols);
    float score_actual = X.at(frames[0]->getID(), frames[1]->getID());

    //Compare
    EXPECT_EQ(score_expected, score_actual);


    // Clear
    Image0.release();
    Image1.release();
    for (unsigned int i = 0; i < frames.size(); i++)
      delete frames[i];  
    frames.clear();

    for (int i = 0; i < GradientStrengths0.size(); i++)
    {
      for (int k = 0; k < GradientStrengths0[i].size(); k++)
      {
        GradientStrengths0[i][k].clear();
        GradientStrengths1[i][k].clear();
      }            
      GradientStrengths0[i].clear();
      GradientStrengths1[i].clear();
    }
    GradientStrengths0.clear();
    GradientStrengths1.clear();

  }


  TEST(ImageHogSimilarityMatrix, frames_ISM)
  {
    //Prepare test data
    int rows = 64, cols = 128;
    int framesCount = 3;
    vector<Frame*> frames;
    for (int i = 0; i < framesCount; i++)
    {
      frames.push_back(new Lockframe());
      frames[i]->setID(i);
      Mat image(rows, cols, CV_8UC3, Scalar(0, 0, 0));
      image.at<Vec3b>(0, 0) = Vec3b(255, 255, 255);
      image.at<Vec3b>(rows - 1, cols - 1) = Vec3b(255, 255, 255);
      cv::ellipse(image, Point(0.5f*cols, 0.5f*rows), Size(0.375f*cols, 0.375f*rows), 0.0, 360.0/(i+1), 360.0, Scalar(255, 255, 255), 1, 0, 0);
      cvtColor(image, image, CV_RGB2GRAY);
      frames[i]->setImage(image);
      frames[i]->setMask(image);
    }

    //Create expected value
    Size winSize(128, 64); Size(0, 1);
    Size blockSize(16, 16);
    Size blockStride(8, 8);
    Size cellSize(8, 8);
    int nBins = 9;
    int derivAper = 0;
    double winSigma = -1.0;
    int histogramNormType = 0;
    double L2HysThresh = 0.2;
    bool gammaCorrection = true;
    int nLevels = 64;
    int N = static_cast<int>(winSize.height / cellSize.height);
    int M = static_cast<int>(winSize.width / cellSize.width);

    //Create expected value
    vector<float> descriptors;
    vector<vector<vector<float>>> GradientStrengths0, GradientStrengths1;
    HOGDescriptor d(winSize, blockSize, blockStride, cellSize, nBins, derivAper, winSigma, histogramNormType, L2HysThresh, gammaCorrection, nLevels);

    Mat SimilarityMatrix(frames.size(), frames.size(), cv::DataType<float>::type, 0.0f);
    for (int i = 0; i < frames.size(); i++)
      for (int k = 0; k < frames.size(); k++)
      {
        d.compute(frames[i]->getImage(), descriptors);
        GradientStrengths0 = averageGradientStrengths(frames[i]->getImage(), descriptors, winSize, blockSize, blockStride, cellSize, nBins, derivAper, winSigma, histogramNormType, L2HysThresh, gammaCorrection, nLevels);
        descriptors.clear();
        d.compute(frames[k]->getImage(), descriptors);
        GradientStrengths1 = averageGradientStrengths(frames[k]->getImage(), descriptors, winSize, blockSize, blockStride, cellSize, nBins, derivAper, winSigma, histogramNormType, L2HysThresh, gammaCorrection, nLevels);
        descriptors.clear();
        float score_expected = 0.0f;
        for (int y = 0; y < N; y++)
          for (int x = 0; x < M; x++)
            for (int n = 0; n < nBins; n++)
              SimilarityMatrix.at<float>(i, k) += pow(GradientStrengths1[y][x][n] - GradientStrengths0[y][x][n], 2);		

        for (int i = 0; i < GradientStrengths0.size(); i++)
        {
          for (int k = 0; k < GradientStrengths0[i].size(); k++)
          {
            GradientStrengths0[i][k].clear();
            GradientStrengths1[i][k].clear();
          }
          GradientStrengths0[i].clear();
          GradientStrengths1[i].clear();
        }
        GradientStrengths0.clear();
        GradientStrengths1.clear();
      }
    ImageHogSimilarityMatrix expected;
    expected.imageSimilarityMatrix = SimilarityMatrix;
    expected.imageShiftMatrix = Mat(frames.size(), frames.size(), cv::DataType<Point2f>::type, 0.0f);
    expected.write("frames_HOG_ISM.txt");

    ImageHogSimilarityMatrix actual;
    try
    {
    //Create actual value
    ImageHogSimilarityMatrix temp(frames);
    actual = temp;
    }
    catch (...)
    {
    }

    //Compare
    EXPECT_EQ(expected, actual);

    SimilarityMatrix.release();

    // Clear
    for (unsigned int i = 0; i < frames.size(); i++)
      delete frames[i];
    frames.clear();
  }

  TEST(ImageHogSimilarityMatrix, extendSize)
  {
    // int8_t ImageHogSimilarityMatrix::extendSize(int & topLeftMin, int & topLeftMax, int & bottomRightMin, int & bottomRightMax, const int maxsize, const int add) const
    // Prepare test data
    int A = 10, B = 40;
    int A1 = 20, B1 = 40;
    int A2 = 0, B2 = 40;
    int A3 = 5, B3 = 15;

    // Create actual value and compare
    ImageHogSimilarityMatrix X;
    int b = X.extendSize(A1, A, B1, B, 100, 0);
    EXPECT_EQ(0, b);
    EXPECT_EQ(15, A1);
    EXPECT_EQ(45, B1);

    b = X.extendSize(A1, A, B1, B, 100, 1);
    EXPECT_EQ(0, b);
    EXPECT_EQ(14, A1);
    EXPECT_EQ(46, B1);

    b = X.extendSize(A, A2, B, B2, 100, 0);
    EXPECT_EQ(0, b);
    EXPECT_EQ(5, A);
    EXPECT_EQ(45, B);

    b = X.extendSize(A3, A, B3, B, 100, 0);
    EXPECT_EQ(0, b);
    EXPECT_EQ(0, A3);
    EXPECT_EQ(40, B3);
  }

}