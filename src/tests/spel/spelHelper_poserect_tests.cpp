// SPEL definitions
#include "predef.hpp"

#include <gtest/gtest.h>
#include <spelHelper.hpp>
#include <bodyPart.hpp>
#include <frame.hpp>
#include "TestsFunctions.hpp"

namespace SPEL
{
  class PoseRectTest : public testing::Test{
  protected:
    //init
    virtual void SetUp(){
      //rectangle points
      p[0] = Point2f(1.0, 2.0);
      p[1] = Point2f(2.0, 2.0);
      p[2] = Point2f(2.0, 1.0);
      p[3] = Point2f(1.0, 1.0);
      p[4] = Point2f(-50.0, -10.0);
      p[5] = Point2f(20.0, -10.0);
      p[6] = Point2f(-50.0, 10.0);
      p[7] = Point2f(20.0, 10.0);
      p[8] = Point2f(4.0, 6.0);
      p[9] = Point2f(9.0, 4.0);
      //test points
      p[10] = Point2f(1.5, 1.5);
      p[11] = Point2f(15.0, 10.0);
      p[12] = Point2f(100.0, 100.0);
      p[13] = Point2f(-20.0f, 9.28f);
      p[14] = Point2f(-15.0, 10.0);
      //first rectangle
      rect1 = POSERECT<Point2f>(p[0], p[1], p[2], p[3]);
      rect3 = POSERECT<Point2f>(p[0], p[1], p[2], p[3]);
      //second rectangle
      rect2 = POSERECT<Point2f>(p[4], p[5], p[6], p[7]);
      rect4 = POSERECT<Point2f>(p[4], p[5], p[6], p[7]);
      //third rectangle
      rect5 = POSERECT<Point2f>(p[6], p[7], p[9], p[8]);

    }

    //clear
    //nothing to clear here
    //virtual void TearDown(){}
  protected:
    Point2f p[15];
    POSERECT<Point2f> rect1, rect2, rect3, rect4, rect5;
  };

  TEST_F(PoseRectTest, Costructor)
  {
    POSERECT<Point2f> rect(p[0], p[1], p[2], p[3]);
    EXPECT_EQ(p[0], rect.point1);
    EXPECT_EQ(p[1], rect.point2);
    EXPECT_EQ(p[2], rect.point3);
    EXPECT_EQ(p[3], rect.point4);
  }

  TEST_F(PoseRectTest, CopyCostructor)
  {
    POSERECT<Point2f> rect(rect1);
    EXPECT_EQ(rect1.point1, rect.point1);
    EXPECT_EQ(rect1.point2, rect.point2);
    EXPECT_EQ(rect1.point3, rect.point3);
    EXPECT_EQ(rect1.point4, rect.point4);
  }

  TEST_F(PoseRectTest, MoveCostructor)
  {
    POSERECT<Point2f> rect(static_cast<POSERECT<Point2f>&&>(rect1));
    EXPECT_EQ(rect1.point1, rect.point1);
    EXPECT_EQ(rect1.point2, rect.point2);
    EXPECT_EQ(rect1.point3, rect.point3);
    EXPECT_EQ(rect1.point4, rect.point4);
  }

  TEST_F(PoseRectTest, ContainsPoint){
    EXPECT_EQ(1, rect1.containsPoint(p[10]));
    EXPECT_EQ(0, rect2.containsPoint(p[11]));
    EXPECT_EQ(-1, rect3.containsPoint(p[12]));
    EXPECT_EQ(1, rect5.containsPoint(p[13]));
    EXPECT_EQ(0, rect5.containsPoint(p[14]));

    EXPECT_EQ(-1, rect1.containsPoint(p[13]));
    EXPECT_EQ(-1, rect3.containsPoint(p[14]));
  }

  TEST_F(PoseRectTest, AsVector){
    std::vector<Point2f> temp = { p[0], p[1], p[2], p[3] };
    EXPECT_EQ(temp, rect1.asVector());
    EXPECT_EQ(4, rect1.asVector().size());
    EXPECT_EQ(temp, rect3.asVector());
    EXPECT_EQ(4, rect3.asVector().size());
    temp = { p[6], p[7], p[9], p[8] };
    EXPECT_EQ(temp, rect5.asVector());
    EXPECT_EQ(4, rect5.asVector().size());
  }

  TEST_F(PoseRectTest, Equality){
    //between rectangles
    EXPECT_TRUE(rect1 == rect3);
    EXPECT_TRUE(rect2 == rect4);
    EXPECT_TRUE(rect1 != rect2);
    EXPECT_TRUE(rect3 != rect4);
    EXPECT_TRUE(rect1 != rect5);
    EXPECT_TRUE(rect2 != rect5);
    EXPECT_TRUE(rect3 != rect5);
    EXPECT_TRUE(rect4 != rect5);
    //between components of rectangle
    EXPECT_EQ(p[0], rect1.point1);
    EXPECT_EQ(p[1], rect1.point2);
    EXPECT_EQ(p[2], rect1.point3);
    EXPECT_EQ(p[3], rect1.point4);

    EXPECT_EQ(p[0], rect3.point1);
    EXPECT_EQ(p[1], rect3.point2);
    EXPECT_EQ(p[2], rect3.point3);
    EXPECT_EQ(p[3], rect3.point4);

    EXPECT_EQ(p[4], rect2.point1);
    EXPECT_EQ(p[5], rect2.point2);
    EXPECT_EQ(p[6], rect2.point3);
    EXPECT_EQ(p[7], rect2.point4);

    EXPECT_EQ(p[4], rect4.point1);
    EXPECT_EQ(p[5], rect4.point2);
    EXPECT_EQ(p[6], rect4.point3);
    EXPECT_EQ(p[7], rect4.point4);

    EXPECT_EQ(p[6], rect5.point1);
    EXPECT_EQ(p[7], rect5.point2);
    EXPECT_EQ(p[9], rect5.point3);
    EXPECT_EQ(p[8], rect5.point4);
  }

  TEST_F(PoseRectTest, assignmentOperator)
  {
    POSERECT<Point2f> rect;
    rect = rect1;
    EXPECT_EQ(rect1, rect);
  }

  TEST_F(PoseRectTest, GetMinMaxXY)
  {
    float Xmin, Xmax, Ymin, Ymax;
    rect5.GetMinMaxXY(Xmin, Ymin, Xmax, Ymax);
    EXPECT_EQ(p[6].x, Xmin);
    EXPECT_EQ(p[7].x, Xmax);
    EXPECT_EQ(p[9].y, Ymin);
    EXPECT_EQ(p[6].y, Ymax);
  }

  TEST_F(PoseRectTest, GetCenter)
  {
    Point2f center = rect1.GetCenter<Point2f>();
    EXPECT_EQ(center, 0.25*(p[0] + p[1] + p[2] + p[3]));
  }

  TEST_F(PoseRectTest, AssignmentOperator)
  {
    POSERECT<Point2f> rect = rect1;
    EXPECT_EQ(rect1, rect);
  }

  TEST_F(PoseRectTest, RectSize)
  {
    Point2f rectSize = rect1.RectSize<Point2f>();
    EXPECT_EQ(Point2f(1.0, 1.0), rectSize);
  }

  TEST(spelHelperTests_, PHPoint)
  {
    float x = 0.5f, y = 0.8f;
    int x_int = static_cast<int>(x), y_int = static_cast<int>(y);
    PHPoint<float> X(x, y);
    EXPECT_EQ(x, X.x);
    EXPECT_EQ(y, X.y);

    PHPoint<float> Y(X);
    EXPECT_EQ(x, Y.x);
    EXPECT_EQ(y, Y.y);

    CvPoint P_int = cvPoint(x_int, y_int);
    PHPoint<int> Z(P_int);
    EXPECT_EQ(x_int, Z.x);
    EXPECT_EQ(y_int, Z.y);

    CvPoint2D32f P_float = cvPoint2D32f(x, y);
    PHPoint<float> Q(P_float);
    EXPECT_EQ(x, Q.x);
    EXPECT_EQ(y, Q.y);

    PHPoint<float> E(cv::Size(x_int, y_int));
    EXPECT_EQ(x_int, E.x);
    EXPECT_EQ(y_int, E.y);

    Vec<float, 2> v = { x, y };
    PHPoint<float> D(v);
    EXPECT_EQ(x, D.x);
    EXPECT_EQ(y, D.y);

    //Operator "<"
    PHPoint<float> W(x - 1.0f, y);
    PHPoint<float> H(x, y - 1.0f);
    EXPECT_TRUE(W < Y);
    EXPECT_TRUE(H < Y);
    EXPECT_FALSE(X < Y);
    EXPECT_FALSE(Y < X);
    EXPECT_FALSE(Y < W);
  }

  TEST(spelHelperTests_, PHPoint3)
  {
    float x = 5.0f, y = 8.0f, z = 7.0f;
    int x_int = static_cast<int>(x), y_int = static_cast<int>(y), z_int = static_cast<int>(z);
    PHPoint<float> P_2d(x, y);

    PHPoint3<float> X(x, y, z);
    EXPECT_EQ(x, X.x);
    EXPECT_EQ(y, X.y);
    EXPECT_EQ(z, X.z);

    PHPoint3<float> Y(X);
    EXPECT_EQ(x, Y.x);
    EXPECT_EQ(y, Y.y);
    EXPECT_EQ(z, Y.z);

    cv::Point3_<float> P(x, y, z);
    PHPoint3<float> Z(P);
    EXPECT_EQ(x, Z.x);
    EXPECT_EQ(y, Z.y);
    EXPECT_EQ(z, Z.z);

    PHPoint3<float> C(P_2d);
    EXPECT_EQ(x, C.x);
    EXPECT_EQ(y, C.y);
    EXPECT_EQ(0.0f, C.z);

    CvPoint3D32f P_float = cvPoint3D32f(x, y,z);
    PHPoint3<float> Q(P_float);
    EXPECT_EQ(x, Q.x);
    EXPECT_EQ(y, Q.y);
    EXPECT_EQ(z, Q.z);

    Vec<float, 3> v = { x, y, z };
    PHPoint3<float> D(v);
    EXPECT_EQ(x, D.x);
    EXPECT_EQ(y, D.y);
    EXPECT_EQ(z, D.z);

    //Operator "<"
    PHPoint3<float> W(x - 1.0f, y, z);
    PHPoint3<float> H(x, y - 1.0f, z);
    PHPoint3<float> F(x, y, z - 1.0f);
    EXPECT_TRUE(W < Y);
    EXPECT_TRUE(H < Y);
    EXPECT_TRUE(F < Y);
    EXPECT_FALSE(X < Y);
    EXPECT_FALSE(Y < X);
    EXPECT_FALSE(Y < W);
  }

  TEST(spelHelperTests_, CopyTree)
  {
    //Loading part tree from existing project
    tree<BodyPart> partTree, copy;
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> frames = project.getFrames();
    partTree = frames[0]->getSkeleton().getPartTree();

    spelHelper::copyTree(copy, partTree);

    ASSERT_EQ(partTree.size(), copy.size());
    
    tree<BodyPart>::pre_order_iterator a, b;
    a = partTree.begin();
    b = copy.begin();
    while (a != partTree.end())
    {
      EXPECT_EQ(*a, *b);
      a++;
      b++;
    }

    /* tree<BodyPart>::breadth_first_iterator c, d;
    c = partTree.begin();
    d = copy.begin();
    while (c != partTree.end())
    {
      EXPECT_EQ(*c, *d);
      c++;
      d++;
    }*/

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    frames.clear();
  }

  TEST(spelHelperTests_, interpolateFloat)
  {
    double prevAngle = 0, nextAngle = 1;
    int numSteps = 5;
    for (int step = 0; step < numSteps+1; step++)
    {
      EXPECT_DOUBLE_EQ(1.0*double(step) / numSteps, spelHelper::interpolateFloat(prevAngle, nextAngle, step, numSteps));
      cout << step << ": " << 1.0*double(step) / numSteps << " ~ " << spelHelper::interpolateFloat(prevAngle, nextAngle, step, numSteps) << endl;
    }

    prevAngle = 1;
    nextAngle = 2;
    for (int step = 0; step < numSteps+1; step++)
      EXPECT_DOUBLE_EQ(1.0 + 1.0*double(step)/double(numSteps), spelHelper::interpolateFloat(prevAngle, nextAngle, step, numSteps));
  }

  TEST(spelHelperTests_, getRandomStr)
  {
    EXPECT_STRNE(spelHelper::getRandomStr().c_str(), spelHelper::getRandomStr().c_str());
  }

  TEST(spelHelperTests_, getGUID)
  {
    EXPECT_STRNE(spelHelper::getGUID().c_str(), spelHelper::getGUID().c_str());
  }

  TEST(spelHelperTests_, getTempFileName)
  {
    EXPECT_STRNE(spelHelper::getTempFileName().c_str(), spelHelper::getTempFileName().c_str());
    string ext = ".txt";
    auto str = spelHelper::getTempFileName(ext);
    EXPECT_STREQ(str.substr(str.length() - ext.length()).c_str(), ext.c_str());
    ifstream i(str);
    EXPECT_FALSE(i.good());
    ofstream of(str, ofstream::out);
    EXPECT_TRUE(of.good());
    EXPECT_NO_THROW({
      of << "Test";
      of.close();
    });
    EXPECT_EQ(remove(str.c_str()), 0);
  }

  TEST(spelHelperTests_, checkFileExists)
  {
    auto str = spelHelper::getTempFileName();
    EXPECT_FALSE(spelHelper::checkFileExists(str));
    ofstream of(str, ofstream::out);
    EXPECT_TRUE(spelHelper::checkFileExists(str));
    EXPECT_NO_THROW({
      of << "Test";
      of.close();
    });
    EXPECT_EQ(remove(str.c_str()), 0);
  }

  TEST(spelHelperTests_, copyFile)
  {
    auto src = spelHelper::getTempFileName();
    ofstream ofs(src);
    ofs << src;
    ofs.close();

    auto dst = spelHelper::getTempFileName();
    EXPECT_NO_THROW(spelHelper::copyFile(dst, src));

    ifstream ifs(dst);
    string ss;
    ifs >> ss;
    ifs.close();

    EXPECT_STREQ(src.c_str(), ss.c_str());
    EXPECT_EQ(remove(src.c_str()), 0);
    EXPECT_EQ(remove(dst.c_str()), 0);
  }
}
