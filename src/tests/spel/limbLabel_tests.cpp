// SPEL definitions
#include "predef.hpp"

#include <gtest/gtest.h>
#include <limbLabel.hpp>
#include <bodyJoint.hpp>
#include "TestsFunctions.hpp"

using namespace std;
using namespace cv;

namespace SPEL
{
  TEST(limbLabel, ConstructorTest)
  {
    int id = 1;
    Point2f center = Point2f(10.0f, 10.0f);
    float angle = 0.866302f;
    bool isOccluded = false;
    float score1Value = 0.1f, score2Value = 0.3f;
    float scoreCoeff = 1.0f;
    Score score1(score1Value, "", scoreCoeff);
    Score score2(score2Value, "", scoreCoeff);
    vector <Score> scores;
    scores.push_back(score1);
    scores.push_back(score2);
    vector<Point2f> polygon = { Point2f(6, 2), Point2f(6, 18), Point2f(14, 18), Point2f(14, 2) };
    Point2f EndPoint1(10, 2), EndPoint2(10, 18);

    //Testing default constructor
    LimbLabel label0;
    EXPECT_EQ(-1, label0.getLimbID());
    EXPECT_EQ(cv::Point2f(0.0f, 0.0f), label0.getCenter());
    EXPECT_EQ(0.0f, label0.getAngle());
    EXPECT_EQ(std::vector<cv::Point2f>({Point2f(0,0), Point2f(0,0), Point2f(0,0), Point2f(0,0) }), label0.getPolygon());
    EXPECT_EQ(std::vector<Score>(), label0.getScores());
    EXPECT_EQ(true, label0.getIsOccluded());

    //Testing constructor with field parameters
    LimbLabel label1(id, center, angle, polygon, scores, isOccluded);
    EXPECT_EQ(id, label1.getLimbID());
    EXPECT_EQ(center, label1.getCenter());
    EXPECT_EQ(angle, label1.getAngle());
    EXPECT_EQ(polygon, label1.getPolygon());
    EXPECT_EQ(scores, label1.getScores());
    EXPECT_EQ(isOccluded, label1.getIsOccluded());

    //Testing constructor with the LimbLabel parameter
    //And testing get() functions for fields 
    LimbLabel label2(label1);
    EXPECT_EQ(label1.getLimbID(), label2.getLimbID());
    EXPECT_EQ(label1.getCenter(), label2.getCenter());
    EXPECT_EQ(label1.getAngle(), label2.getAngle());
    EXPECT_EQ(label1.getPolygon(), label2.getPolygon());
    EXPECT_EQ(label1.getScores(), label2.getScores());
    EXPECT_EQ(label1.getIsOccluded(), label2.getIsOccluded());

    //Testing function getAvgScore()
    bool bNegativeToPositive = true;
    //float avgScore = (score1Value + score2Value)*scoreCoeff / 2;
    float avgScore = (score1Value + score2Value)*scoreCoeff; // Use sum instead of average
    // Temporary debug messages
    if (avgScore != label1.getAvgScore(bNegativeToPositive))
      cout << "-----------\n   LimbLabel::GetAvgScore():\n" << endl;
    //
    EXPECT_EQ(avgScore, label1.getAvgScore(bNegativeToPositive));
    // Temporary debug messages
    if (avgScore != label1.getAvgScore(bNegativeToPositive))
    {
      cout << endl;
      cout << "----------\n" << endl;
      //cin.get();
    }
    //

    //Testing function AddScore()
    LimbLabel label3;
    label3 = label1;
    float score3Value = 0.1f;
    Score score3(score3Value, "", scoreCoeff);
    scores.push_back(score3);
    label3.addScore(score3);
    EXPECT_EQ(scores, label3.getScores());

    //Testing function getEndPoints()
    Point2f p1, p2;
    label1.getEndpoints(p1, p2);
    EXPECT_EQ(EndPoint1, p1);
    EXPECT_EQ(EndPoint2, p2);
    //    0---3
    //    |   |
    //    1---2

    //Testing function containsPoint()
    //bool b = label1.containsPoint(center);
    EXPECT_TRUE(label1.containsPoint(center));
    EXPECT_FALSE(label1.containsPoint(Point2f(-1.0, -1.0)));

  }

  TEST(limbLabel, Operators)
  {
    int id = 1;
    Point2f center = Point2f(10.0f, 10.0f);
    float angle = 0.866302f;
    bool isOccluded = false;
    float score1Value = 0.1f, score2Value = 0.3f;
    float scoreCoeff = 1.0f;
    Score score1(score1Value, "", scoreCoeff);
    Score score2(score2Value, "", scoreCoeff);
    vector <Score> scores;
    scores.push_back(score1);
    scores.push_back(score1);
    vector<Point2f> polygon = { Point2f(6, 2), Point2f(6, 18), Point2f(14, 18), Point2f(14, 2) };
    LimbLabel label1(id, center, angle, polygon, scores, isOccluded);
    scores.clear();
    scores.push_back(score2);
    scores.push_back(score2);
    LimbLabel label2(id, center, angle, polygon, scores, isOccluded);
    LimbLabel label3(label1);
    LimbLabel label4(id + 1, center, angle, polygon, scores, isOccluded);
    LimbLabel label5(id, center + Point2f(0.1f, 0.0f), angle, polygon, scores, isOccluded);
    LimbLabel label6(id, center, angle+0.1f, polygon, scores, isOccluded);

    //Testing operator "="
    EXPECT_EQ(label1.getLimbID(), label3.getLimbID());
    EXPECT_EQ(label1.getCenter(), label3.getCenter());
    EXPECT_EQ(label1.getAngle(), label3.getAngle());
    EXPECT_EQ(label1.getPolygon(), label3.getPolygon());
    EXPECT_EQ(label1.getScores(), label3.getScores());
    EXPECT_EQ(label1.getIsOccluded(), label3.getIsOccluded());

    //Testing operators "=="
    EXPECT_TRUE(label1 == label3);
    EXPECT_FALSE(label1 == label4);
    EXPECT_FALSE(label1 == label5);
    EXPECT_FALSE(label1 == label6);

    //Testing operators "!="
    EXPECT_FALSE(label1 != label3);
    EXPECT_TRUE(label1 != label4);
    EXPECT_TRUE(label1 != label5);
    EXPECT_TRUE(label1 != label6);

    //Testing operator "<"
    EXPECT_TRUE(label1 < label2) << " Operator '<', expected: " << label1.getAvgScore() << " < " << label2.getAvgScore() << " = true" << endl;
    EXPECT_FALSE(label2 < label1) << " Operator '<', expected: " << label2.getAvgScore() << " < " << label1.getAvgScore() << " = false" << endl;
    EXPECT_FALSE(label1 < label1) << " Operator '<', expected: " << label1.getAvgScore() << " < " << label1.getAvgScore() << " = false" << endl;

    //Testing operator ">"
    EXPECT_TRUE(label2 > label1) << " Operator '>', expected: " << label2.getAvgScore() << " > " << label1.getAvgScore() << " = true" << endl;
    EXPECT_FALSE(label1 > label2) << " Operator '>', expected: " << label1.getAvgScore() << " > " << label2.getAvgScore() << " = false" << endl;
    EXPECT_FALSE(label1 > label1) << " Operator '>', expected: " << label1.getAvgScore() << " > " << label1.getAvgScore() << " = false" << endl;
  }

  TEST(limbLabel, getEndpoints)
  {
    //Prepare test data
    int id = 0;
    float lwRatio = 3.0f;
    Point2f p0(20.0f, 40.0f), p1(20.0f, 20.0f);
    BodyJoint* j0 = new BodyJoint(id, "j0", p0, Point3f(p0.x, p0.y, 0.0f), false);
    BodyJoint* j1 = new BodyJoint(id, "j1", p1, Point3f(p1.x, p1.y, 0.0f), false);
    vector<Point2f> polygon = BuildPartRect(j0, j1, lwRatio).asVector();
    Point2f center = 0.5f*(p0 + p1);
    float angle = 0.0f;
    bool isOccluded = false;
    vector <Score> scores;
    LimbLabel label0(id, center, angle, polygon, scores, isOccluded);

    //Create actual value
    Point2f p0_actual, p1_actual;
    label0.getEndpoints(p0_actual, p1_actual);

    //Compare
    EXPECT_FLOAT_EQ(p0.x, p0_actual.x);
    EXPECT_FLOAT_EQ(p0.y, p0_actual.y);
    EXPECT_FLOAT_EQ(p1.x, p1_actual.x);
    EXPECT_FLOAT_EQ(p1.y, p1_actual.y);

    polygon.clear();
  }

  TEST(limbLabel, getCenter)
  {
    //Prepare test data
    int id = 0;
    Point2f p0(20.0f, 40.0f), p1(20.0f, 20.0f), d(6.0f, 0.0f);
    vector<Point2f> polygon = {p0 + d, p1+d, p1-d, p0-d};
    Point2f center = 0.5f*(p0 + p1);
    float angle = 0.0f;
    bool isOccluded = false;
    vector <Score> scores;
    LimbLabel label0(id, center, angle, polygon, scores, isOccluded);

    //Create actual value and compare
    EXPECT_EQ(center, label0.getCenter());

    polygon.clear();
  }

  TEST(limbLabel, GetAndSetScores)
  {
    //Prepare test data
    int id = 0;
    Point2f p0(20.0f, 40.0f), p1(20.0f, 20.0f), d(6.0f, 0.0f);
    vector<Point2f> polygon = {p0 + d, p1+d, p1-d, p0-d};
    Point2f center = 0.5f*(p0 + p1);
    float angle = 0.0f;
    bool isOccluded = false;
    float score1Value = 0.1f, score2Value = 0.3f;
    float scoreCoeff = 1.0f;
    Score score1(score1Value, "", scoreCoeff);
    Score score2(score2Value, "", scoreCoeff);
    vector <Score> scores0, scores1;
    scores0.push_back(score1);
    scores0.push_back(score1);  
    LimbLabel label0(id, center, angle, polygon, scores0, isOccluded);

    //Create actual value and compare
    EXPECT_EQ(scores0, label0.getScores());
    scores1.push_back(score1);
    scores1.push_back(score2);
    label0.setScores(scores1);
    EXPECT_EQ(scores1, label0.getScores());

    scores0.clear();
    scores1.clear();
    polygon.clear();
  }

  TEST(limbLabel, getLimbID)
  {
    //Prepare test data
    int id = 50;
    vector<Point2f> polygon;
    Point2f center = Point2f(0.0f, 0.0f);
    float angle = 0.0f;
    bool isOccluded = false;
    vector <Score> scores;
    LimbLabel label0(id, center, angle, polygon, scores, isOccluded);

    //Create actual value and compare
    EXPECT_EQ(id, label0.getLimbID());
  }

  TEST(limbLabel, getAngle)
  {
    //Prepare test data
    int id = 50;
    vector<Point2f> polygon;
    Point2f center = Point2f(0.0f, 0.0f);
    float angle = 90.0f;
    bool isOccluded = false;
    vector <Score> scores;
    LimbLabel label0(id, center, angle, polygon, scores, isOccluded);

    //Create actual value and compare
    EXPECT_EQ(angle, label0.getAngle());
  }

  TEST(limbLabel, getPolygon)
  {
    //Prepare test data
    int id = 0;
    Point2f p0(20.0f, 40.0f), p1(20.0f, 20.0f), d(6.0f, 0.0f);
    vector<Point2f> polygon = {p0 + d, p1+d, p1-d, p0-d};
    Point2f center = 0.5f*(p0 + p1);
    float angle = 0.0f;
    vector <Score> scores;
    bool isOccluded = false;
    LimbLabel label0(id, center, angle, polygon, scores, isOccluded);

    //Create actual value and compare
    EXPECT_EQ(polygon, label0.getPolygon());

    polygon.clear();
  }

  TEST(limbLabel, getIsOccluded)
  {
    //Prepare test data
    int id = 0;
    Point2f p0(20.0f, 40.0f), p1(20.0f, 20.0f), d(6.0f, 0.0f);
    vector<Point2f> polygon = {p0 + d, p1+d, p1-d, p0-d};
    Point2f center = 0.5f*(p0 + p1);
    float angle = 0.0f;
    vector <Score> scores;
    bool isOccluded = false;
    LimbLabel label0(id, center, angle, polygon, scores, isOccluded);
    LimbLabel label1(id, center, angle, polygon, scores, !isOccluded);

    //Create actual value and compare
    EXPECT_EQ(isOccluded, label0.getIsOccluded());
    EXPECT_EQ(!isOccluded, label1.getIsOccluded());

    polygon.clear();
  }

  TEST(limbLabel, containsPoint)
  {
    //Prepare test data
    int id = 0;
    Point2f p0(20.0f, 40.0f), p1(20.0f, 20.0f), d(6.0f, 0.0f);
    vector<Point2f> polygon = {p0 + d, p1+d, p1-d, p0-d};
    Point2f center = 0.5f*(p0 + p1);
    float angle = 0.0f;
    bool isOccluded = false;
    vector <Score> scores;

    LimbLabel label0(id, center, angle, polygon, scores, isOccluded);

    //Create actual value and compare
    EXPECT_TRUE(label0.containsPoint(center));
    EXPECT_TRUE(label0.containsPoint(p0));
    EXPECT_TRUE(label0.containsPoint(p1));
    EXPECT_TRUE(label0.containsPoint(center - d));
    EXPECT_FALSE(label0.containsPoint(p0 + Point2f(0.0f, 1.0f)));
    EXPECT_FALSE(label0.containsPoint(p0 - d - Point2f(1.0f, 0.0f)));
    EXPECT_FALSE(label0.containsPoint(p1 + d + Point2f(1.0f, 0.0f)));
    EXPECT_FALSE(label0.containsPoint(p1 - Point2f(0.0f, 1.0f)));

    polygon.clear();
  }

  TEST(limbLabel, ToString)
  {
    //Prepare input data
    int id = 1;
    Point2f center = Point2f(10.0f, 10.0f);
    float angle = 0.866302f;
    bool isOccluded = false;
    float score1Value = 0.1f, score2Value = 0.3f;
    float scoreCoeff = 1.0f;
    Score score1(score1Value, "Test", scoreCoeff);
    Score score2(score2Value, "Test", scoreCoeff);
    vector <Score> scores;
    scores.push_back(score1);
    scores.push_back(score2);
    vector<Point2f> polygon = { Point2f(6, 2), Point2f(6, 18), Point2f(14, 18), Point2f(14, 2) };
    LimbLabel label(id, center, angle, polygon, scores, isOccluded);

    //Compare
    EXPECT_EQ("1 10.000000 10.000000 0.866302 10.000000 2.000000 10.000000 18.000000 0 Test 0.100000 Test 0.300000 ", label.toString());

    scores.clear();
    polygon.clear();
  }

  TEST(limbLabel, resize)
  {
    //Prepare input data
    int id = 1;
    Point2f center = Point2f(10.0f, 10.0f);
    float angle = 0.866302f;
    bool isOccluded = false;
    float score1Value = 0.1f, score2Value = 0.3f;
    float scoreCoeff = 1.0f;
    Score score1(score1Value, "", scoreCoeff);
    Score score2(score2Value, "", scoreCoeff);
    vector <Score> scores;
    scores.push_back(score1);
    scores.push_back(score1);
    Point2f A(6.0f, 2.0f), B(6.0f, 18.0f), C(14.0f, 18.0f), D(14.0f, 2.0f);
    vector<Point2f> polygon = { A, B, C, D };
    LimbLabel label(id, center, angle, polygon, scores, isOccluded);
    float factor = 10.0f;
    vector<Point2f> expected_polygon = { A*factor, B*factor, C*factor, D*factor };

    //Compare
    label.Resize(factor);
    EXPECT_EQ(center*10.0f, label.getCenter());
    EXPECT_EQ(expected_polygon, label.getPolygon());

    scores.clear();
    polygon.clear();
  }




}
