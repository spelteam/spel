#include <gtest/gtest.h>
#include <colorHistDetector.hpp>
#include <tree.hh>
#include "bodyPart.hpp"
#include "Skeleton.hpp"
#include "Keyframe.hpp"
#include "Lockframe.hpp"
#include <fstream>
#include <iostream>
#include "frame.hpp"
#include <string>
#include <interpolation.hpp>
#include "projectLoader.hpp"
#include <fstream>

class CHD : public ColorHistDetector
{
public:
    vector <vector <vector <float>>> GetPartHistogramm(int Part_id);
};

// Colorspace scaling coefficientModel.nBins
const uint8_t factor = static_cast<uint8_t> (ceil(pow(2, 8) / 8));

//Built bodypart rectangle on bodypart joints
POSERECT<Point2f> BuildPartRect(BodyJoint *j0, BodyJoint *j1, float LWRatio)
{
    Point2f p0 = j0->getImageLocation(), p1 = j1->getImageLocation();
    float boneLength = (float)sqrt(PoseHelper::distSquared(p0, p1)); // distance between nodes
    float boneWidth = boneLength / LWRatio;
    Point2f boxCenter = p0 * 0.5 + p1 * 0.5; // the bobypart center  coordinates
    // Coordinates for drawing of the polygon at the coordinate origin
    Point2f c1 = Point2f(0.f, 0.5f * boneWidth);
    Point2f c2 = Point2f(boneLength, 0.5f * boneWidth);
    Point2f c3 = Point2f(boneLength, -0.5f * boneWidth);
    Point2f c4 = Point2f(0.f, -0.5f * boneWidth);
    Point2f polyCenter = Point2f(boneLength * 0.5f, 0.f); // polygon center 
    Point2f direction = p1 - p0; // used as estimation of the vector's direction
    float rotationAngle = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI)); //bodypart tilt angle 
    // Rotate and shift the polygon to the bodypart center
    c1 = PoseHelper::rotatePoint2D(c1, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c2 = PoseHelper::rotatePoint2D(c2, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c3 = PoseHelper::rotatePoint2D(c3, polyCenter, rotationAngle) + boxCenter - polyCenter;
    c4 = PoseHelper::rotatePoint2D(c4, polyCenter, rotationAngle) + boxCenter - polyCenter;
    POSERECT <Point2f> poserect(c1, c2, c3, c4);
    return poserect;
}

//Return "true" if polygon is crossed (occluded) by "rect"
bool IsCrossed(vector<Point2f> PolygonPoints, POSERECT<Point2f> rect)
{
    bool Crossed = 0;
    for (int i = 0; i < PolygonPoints.size() - 1; i++)
    {
        int k = i;
        if (k == PolygonPoints.size() - 1)
            k = 0;
        Point2f delta = PolygonPoints[i + 1] - PolygonPoints[k];
        float n = 2 * max(abs(delta.x), abs(delta.y));
        float dx = delta.x / n;
        float dy = delta.y / n;
        float x = PolygonPoints[i].x, y = PolygonPoints[i].y;
        for (int k = 0; k < n; k++)
        {
            Crossed = Crossed || (rect.containsPoint(Point2f(x, y))>0);
            y = y + dy;
            x = x + dx;
        }
    }
    return Crossed;
}

//Return "true" if "rect1" is crossed (occluded) by "rect2"
bool IsCrossed(POSERECT<Point2f> rect1, POSERECT<Point2f> rect2)
{
    vector<Point2f> Polygon = rect1.asVector();
    bool Crossed = IsCrossed(Polygon, rect2);
    Polygon.clear();
    return Crossed;
}

class depthCompare
{
public:
    bool operator () (pair<int, int> X, pair<int, int> Y)
    {
        return Y.second > X.second;
    }
};

//For the each polygon select all polygons, which crossed it 
vector<vector<pair<int, int>>> CrossingsList(map<int, POSERECT<Point2f>> Rects, map<int, int> depth)
{   
    vector<vector<pair<int, int>>> Crosses;
    for (int i = 0; i < Rects.size(); i++)
    {
        vector<pair<int, int>> X;
        vector<Point2f> Polygon = Rects[i].asVector();
        for (int k = 0; k < Rects.size(); k++)
        if (depth[k] < depth[i])
        {
            if (IsCrossed(Polygon, Rects[k]))
                X.push_back(pair<int, int>(k, depth[k]));
        }
        sort(X.begin(), X.end(), depthCompare());
        Crosses.push_back(X);
        X.clear();
        Polygon.clear();
    }
    return Crosses;
}

void  Normalize(vector <vector <vector <float>>> &Histogramm, int nBins, int pixelsCount)
{
    for (int r = 0; r < nBins; r++)
    for (int g = 0; g < nBins; g++)
    for (int b = 0; b < nBins; b++)
        Histogramm[b][g][r] = Histogramm[b][g][r] / pixelsCount;
}

void PutHistogramm(ofstream &fout, vector <vector <vector <float>>> &Histogramm, int sizeFG)
{
    int nBins = 8;
    for (int r = 0; r < nBins; r++)
    for (int g = 0; g < nBins; g++)
    for (int b = 0; b < nBins; b++)
    if (Histogramm[b][g][r]>0)
        fout << "Histogram[" << r << "," << g << "," << b << "] = " << Histogramm[b][g][r] * sizeFG << ";\n";
}

TEST(colorHistDetectorTest, Train)
{
    String FilePath; 

    #ifdef WINDOWS
        FilePath = "Debug/posetests_TestData/CHDTrainTestData/";
    #else
        FilePath = "Release/posetests_TestData/CHDTrainTestData/";
    #endif

//Load the input data
    ProjectLoader projectLoader(FilePath);
    projectLoader.Load(FilePath + "trijumpSD_50x41.xml");
    Mat image = imread(FilePath + "seq/0000_50x41.png");
    vector<Frame*> frames = projectLoader.getFrames();

//Counting a keyframes
    int KeyframesCount = 0;
    int FirstKeyframe = -1;
    for (int i = 0; i < frames.size();i++)
    if (frames[i]->getFrametype() == KEYFRAME)
    {
        KeyframesCount++;
        if (FirstKeyframe < 0) FirstKeyframe = i;
    }

//Ran "Train()"
    map <string, float> params;
    ColorHistDetector detector;
    detector.train(frames, params);

//Copy skeleton from keyframe
    Frame *frame = frames[FirstKeyframe];
    Skeleton skeleton = frame->getSkeleton();
    tree<BodyPart> PartTree = skeleton.getPartTree();

//Build the rectangles for all of bodyparts
    map<int, POSERECT<Point2f>> Rects;
    for (tree<BodyPart>::iterator BP_iterator = PartTree.begin(); BP_iterator != PartTree.end(); BP_iterator++)
    {
        BodyJoint *j0 = skeleton.getBodyJoint(BP_iterator->getChildJoint());
        BodyJoint *j1 = skeleton.getBodyJoint(BP_iterator->getParentJoint());
        POSERECT<Point2f> Rect = BuildPartRect(j0, j1, BP_iterator->getLWRatio());
        Rects.emplace(pair<int, POSERECT<Point2f>>((*BP_iterator).getPartID(), Rect));
    }

//Calculate the polygons occluded
    //Polygons layers:
    map<int, int> depth = { { 0, 2 }, { 1, 1 }, { 2, 3 }, { 3, 2 }, { 4, 4 }, { 5, 4 }, { 6, 1 }, { 7, 3 }, { 8, 2 }, { 9, 0 }, { 10, 4 }, { 11, 1 }, { 12, 3 }, { 13, 0 }, { 14, 4 }, { 15, 1 }, { 16, 3 } };    
    //Polygons occluded:
    vector<vector<pair<int, int>>> Crossings = CrossingsList(Rects, depth);

//Calculate the parts histogramms
    map <int32_t, ColorHistDetector::PartModel> partModels;
    for (int i = 0; i < Rects.size(); i++)
    {
        ColorHistDetector::PartModel Model(8);
        Model.sizeFG = 0;
        float xmin, ymin, xmax, ymax;
        Rects[i].GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
        for (int x = xmin; x < xmax; x++)
        {
            for (int y = ymin; y < ymax; y++)
            {
                bool b = true;
                if (Rects[i].containsPoint(Point2f(x, y)) > 0)
                {
                    int k = 0;

                    while ((k < Crossings[i].size()) && b)
                    {
                        if (Rects[Crossings[i][k].first].containsPoint(Point2f(x, y)) > 0)
                            b = false;
                        k++;
                    }
                    if (b)
                    {
                        Vec3b color = image.at<Vec3b>(y, x);
                        int c = 50 + i * 10;
                        image.at<Vec3b>(y, x) = Vec3b(c, c, c);
                        Model.partHistogramm[color[0] / factor][color[1] / factor][color[2] / factor]++;
                        Model.sizeFG++;
                    }
                }
            }
        }
        partModels.emplace(pair<int32_t, ColorHistDetector::PartModel>(i, Model));
    }

//Put results
    int nBins = detector.nBins;
    bool AllValuesEqual = true;
    int delta = 2; // tolerable linear error

    ofstream fout("TrainUnitTest_Output.txt");
    fout << "\n--------------------------Don't equal----------------------\n";
    cout << "\nTolerable error: " << delta << endl;
    fout << "Tolerable error: " << delta << endl;
    for (int i = 0; i < partModels.size(); i++)
    {
        for (int r = 0; r < nBins; r++)
        for (int g = 0; g < nBins; g++)
        for (int b = 0; b < nBins; b++)
        {
             int expected = int(partModels[i].partHistogramm[b][g][r]);
             int actual = int(detector.partModels[i].partHistogramm[r][g][b] * detector.partModels[i].sizeFG / KeyframesCount);
             if (abs(expected - actual) > delta)
             {
                 cout <<"Part[" << i << "]." << "Histogram[" << r << ", " << g << ", " << b << "]:    Expected = " << expected << ",   Actual = " << actual << endl;
                 fout << "Part[" << i << "]." << "Histogram[" << r << ", " << g << ", " << b << "]:    Expected = " << expected << ",   Actual = " << actual << endl;
                 AllValuesEqual = false;
             }
        }
    }
    if (AllValuesEqual) fout << "none";

    cout << "Out files: TrainUnitTest_Output.txt, UsedPixels.png\n\n";
    EXPECT_TRUE(AllValuesEqual);

    fout << "\n-----------Expected histogramm-----------\n";
    fout << "In format:\nHistogramm[r, g, b] = pixelsCount\n";
    for (int i = 0; i < partModels.size(); i++)
    {
        fout << endl << "Rect[" << i << "]:" << endl;
        PutHistogramm(fout, partModels[i].partHistogramm, 1);
    }

    fout << "\n-----------Actual histogramm-----------\n";
    fout << "In format:\nHistogramm[b, g, r] = Histogramm[b, g, r]*Part.SizeFG/KeyframesCout\n";
    for (int i = 0; i < detector.partModels.size(); i++)
    {
        fout << endl << "Rect[" << i << "]:" << endl;
        PutHistogramm(fout, detector.partModels[i].partHistogramm, detector.partModels[i].sizeFG);
    }

    fout << "\n------------Overlapping polygons-----------\nSorted by layer\n";
    for (int i = 0; i < Crossings.size(); i++)
    {
        fout << "\nPolygon[" << i << "] crossed by polygons: ";
        for (int k = 0; k < Crossings[i].size(); k++)
            fout << Crossings[i][k].first << "; ";
        Crossings[i].clear();
    }
    imwrite("UsedPixels.png", image);
    fout.close();
    Crossings.clear();
    partModels.clear();

    image.release();
}