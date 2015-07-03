#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

#include <gtest/gtest.h>
#include <detector.hpp>
#include <surfDetector.hpp>
#include "projectLoader.hpp"
#include "limbLabel.hpp"

void PutPartRect(Mat &Image, vector<Point2f> polygon, Scalar color)
{
    polygon.push_back(polygon[0]);
    for (int i = 1; i < polygon.size(); i++)
        line(Image, polygon[i - 1], polygon[i], color, 1, 1);
}

vector <KeyPoint> SelectKeypoints(vector <KeyPoint> FrameKeypoints, POSERECT<Point2f> rect)
{
    vector <KeyPoint> keyPoints;
    for (int i = 0; i < FrameKeypoints.size(); i++)
      if (rect.containsPoint(FrameKeypoints[i].pt) > 0)
        keyPoints.push_back(FrameKeypoints[i]);
    return keyPoints;
}

//Testing "computeDescriptors" functions
TEST(surfDetectorTests, computeDescriptors)
{
    String FilePath;
    FilePath = "posetests_TestData/SurfDetectorTestsData/";

#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
        FilePath = "Debug/posetests_TestData/SurfDetectorTestsData/";
#endif

    //Load the input data
    ProjectLoader projectLoader(FilePath);
    projectLoader.Load(FilePath + "trijumpSD_shortcut.xml");
    vector<Frame*> frames = projectLoader.getFrames();

    //Counting a keyframes
    int KeyframesCount = 0;
    int FirstKeyframe = -1;
    for (int i = 0; i < frames.size(); i++)
      if (frames[i]->getFrametype() == KEYFRAME)
      {
        KeyframesCount++;
        if (FirstKeyframe < 0) FirstKeyframe = i;
      }

    //Copy image and skeleton from first keyframe
    Mat image = frames[FirstKeyframe]->getImage();
    Mat mask = frames[FirstKeyframe]->getMask();
    Skeleton skeleton = frames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    //Select body part for testing
    const int  partID = 6;
    //Copy body part
    BodyPart bodyPart = *skeleton.getBodyPart(partID);
    //Copy part joints 
    BodyJoint* j0 = skeleton.getBodyJoint(bodyPart.getParentJoint());
    BodyJoint* j1 = skeleton.getBodyJoint(bodyPart.getChildJoint());
    //Copy joints location
    Point2f p0 = j0->getImageLocation();
    Point2f p1 = j1->getImageLocation();

    //Calculate actual value
    uint32_t minHessian = 500;
    SurfDetector D;
    map <uint32_t, SurfDetector::PartModel> actual_PartModels = D.computeDescriptors(frames[FirstKeyframe], minHessian);
    SurfDetector::PartModel actual_PartModel = D.computeDescriptors(bodyPart, p0, p1, image, minHessian, actual_PartModels[partID].keyPoints);

//Calculate expected values

    // Part rect
    float boneLength = D.getBoneLength(p0, p1);
    float boneWidth = D.getBoneWidth(boneLength, bodyPart);
    POSERECT <Point2f> rect = D.getBodyPartRect(bodyPart, p0, p1, Size(boneLength, boneWidth)); 
    //Frame keypoints
    vector <KeyPoint> expected_FrameKeyPoints; 
#if OpenCV_VERSION_MAJOR == 3
    Ptr <SurfFeatureDetector> D1 = SurfFeatureDetector::create(minHessian);
    D1->detect(image, expected_FrameKeyPoints);
#else
    SurfFeatureDetector D1(minHessian);
    D1.detect(image, expected_FrameKeyPoints);
#endif
    //Part keypoints
    vector <KeyPoint> expected_PartKeyPoints = SelectKeypoints(expected_FrameKeyPoints, rect);

    //Part descriptors
    Mat expected_PartDescriptors;
#if OpenCV_VERSION_MAJOR == 3
    Ptr <SurfDescriptorExtractor> extractor = SurfDescriptorExtractor::create();
    extractor->compute(image, expected_PartKeyPoints, expected_PartDescriptors);
#else
    SurfDescriptorExtractor extractor;
    extractor.compute(image, expected_PartKeyPoints, expected_PartDescriptors);
#endif
//Compare

    //Compare part rect
    EXPECT_EQ(rect.asVector(), actual_PartModels[partID].partModelRect.asVector());

    //Compare keypoints size
    EXPECT_EQ(expected_PartKeyPoints.size(), actual_PartModels[partID].keyPoints.size());

    //Compare current part keypoints
    for (int i = 0; i < min(expected_PartKeyPoints.size(), actual_PartModels[partID].keyPoints.size()); i++)
      EXPECT_EQ(expected_PartKeyPoints[i].pt, actual_PartModels[partID].keyPoints[i].pt);

    //Compare current part model descriptrors
    bool All_values_equal = true;
    Size expected_descriptorsSize = expected_PartDescriptors.size();
    Size actual_descriptorsSize = actual_PartModels[partID].descriptors.size();
    EXPECT_EQ(expected_descriptorsSize, actual_descriptorsSize) << "Size of partmodel descriptors";
    for (int i = 0; i < min(expected_descriptorsSize.height, actual_descriptorsSize.height); i++)
      for (int k = 0; k < min(expected_descriptorsSize.width, actual_descriptorsSize.width); k++)
        All_values_equal = expected_PartDescriptors.at<float>(i, k) == actual_PartModels[partID].descriptors.at<float>(i, k);
    EXPECT_TRUE(All_values_equal) << "descriptors";

//Put expected and actual polygons and keypoints into image file
    Mat KeyPoints_image, PartKeyPoints, expected_KeyPoints;
    drawKeypoints(image, expected_FrameKeyPoints, KeyPoints_image, 128);
    imwrite("KeyPoints_Frame.jpg", KeyPoints_image);

    drawKeypoints(image, actual_PartModels[partID].keyPoints, PartKeyPoints, 128);
    PutPartRect(PartKeyPoints, actual_PartModel.partModelRect.asVector(), 255);
    imwrite("KeyPoints_CurrentPart.jpg", PartKeyPoints);

    drawKeypoints(image, expected_PartKeyPoints, expected_KeyPoints, 128);
    PutPartRect(expected_KeyPoints, rect.asVector(), Scalar(0, 255, 0));
    imwrite("KeyPoints_Expected.jpg", expected_KeyPoints);

    cout << "See keypoints in files: KeyPoints_Frame.jpg, KeyPoints_CurrentPart.jpg, KeyPoints_Expected.jpg" << endl;

    KeyPoints_image.release();
    PartKeyPoints.release();
    expected_KeyPoints.release();
}


TEST(surfDetectorTests, train)
{
    String FilePath;
    FilePath = "posetests_TestData/SurfDetectorTestsData/";

#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
        FilePath = "Debug/posetests_TestData/SurfDetectorTestsData/";
#endif

    //Load the input data
    ProjectLoader projectLoader(FilePath);
    projectLoader.Load(FilePath + "trijumpSD_shortcut.xml");
    vector<Frame*> frames = projectLoader.getFrames();

    //Counting a keyframes
    int KeyframesCount = 0;
    int FirstKeyframe = -1;
    for (int i = 0; i < frames.size(); i++)
      if (frames[i]->getFrametype() == KEYFRAME)
        {
          KeyframesCount++;
          if (FirstKeyframe < 0) FirstKeyframe = i;
        }

    //Copy image and skeleton from first keyframe
    Mat image = frames[FirstKeyframe]->getImage();
    Mat mask = frames[FirstKeyframe]->getMask();
    Skeleton skeleton = frames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();
    Size ImageSize = image.size();

    //Calculate actual value
    uint32_t minHessian = 500;
    SurfDetector D;
    map <string, float> params;
    params.emplace(pair<string, float>("maxFrameHeight", ImageSize.height));
    D.train(frames, params);

    //Calculate expected value
    SurfFeatureDetector D1(minHessian);
    vector <KeyPoint> keyPoints;
    D1.detect(image, keyPoints);
    map<uint32_t, Mat> expected_descriptors;
    map<uint32_t, vector<Point2f>> expected_rects;
    map<uint32_t, vector<KeyPoint>> expected_Keypoints;
    for (tree <BodyPart>::iterator part = partTree.begin(); part != partTree.end(); part++)
    {
      int partID = part->getPartID();
      vector<KeyPoint> temp;

      BodyJoint* j0 = skeleton.getBodyJoint(part->getParentJoint());
      BodyJoint* j1 = skeleton.getBodyJoint(part->getChildJoint());
      Point2f p0 = j0->getImageLocation();
      Point2f p1 = j1->getImageLocation();
      float boneLength = D.getBoneLength(p0, p1);
      float boneWidth = D.getBoneWidth(boneLength, *part);
      POSERECT<Point2f>  rect = D.getBodyPartRect(*part, p0, p1, Size(boneLength, boneWidth));
      expected_rects[partID] = rect.asVector();

      for (int i = 0; i < keyPoints.size(); i++)
        if (rect.containsPoint(keyPoints[i].pt) > 0)
          temp.push_back(keyPoints[i]);
      expected_Keypoints.emplace(pair<uint32_t, vector<KeyPoint>>(partID, temp));

      SurfDescriptorExtractor extractor;
      Mat temp_descriptors;
      extractor.compute(image, expected_Keypoints[partID], temp_descriptors);
      expected_descriptors.emplace(pair<uint32_t, Mat>(partID, temp_descriptors.clone()));
      temp_descriptors.release();
      temp.clear();
    }

    //Compare
    vector<Point2f> empty_rect = { Point2f(0, 0), Point2f(0, 0), Point2f(0, 0), Point2f(0, 0) };
    for (int f = 0; f < frames.size(); f++)
      for (tree<BodyPart>::iterator p = partTree.begin(); p != partTree.end(); p++)
        {
          int partID = p->getPartID();
          if (frames[f]->getFrametype() != INTERPOLATIONFRAME)
            {
              EXPECT_EQ(expected_rects[partID], D.partModels[f][partID].partModelRect.asVector());
              for (int i = 0; i < expected_Keypoints[partID].size(); i++)
                EXPECT_EQ(expected_Keypoints[partID][i].pt, D.partModels[f][partID].keyPoints[i].pt);
              bool All_values_equal = true;
              Size size = expected_descriptors[partID].size();
              for (int i = 0; i < size.height; i++)
                for (int k = 0; k < size.width; k++)
                  All_values_equal = expected_descriptors[partID].at<float>(i, k) == D.partModels[f][partID].descriptors.at<float>(i, k);
              EXPECT_TRUE(All_values_equal);
            }
            if (frames[f]->getFrametype() == INTERPOLATIONFRAME)
            {
              EXPECT_EQ(empty_rect, D.partModels[f][partID].partModelRect.asVector()); //?
              EXPECT_TRUE(D.partModels[f][partID].keyPoints.empty());
              for (int i = 0; i < D.partModels[f][partID].keyPoints.size(); i++)
                cout << D.partModels[f][partID].keyPoints[i].pt << endl;
            }
            expected_Keypoints[partID].clear();
          }

    expected_rects.clear();
}

//Temporary test
TEST(surfDetectorTests, compare)
{
    String FilePath;
    FilePath = "posetests_TestData/SurfDetectorTestsData/";

#if defined(WINDOWS) && defined(_MSC_VER)
    if (IsDebuggerPresent())
        FilePath = "Debug/posetests_TestData/SurfDetectorTestsData/";
#endif

    //Load the input data
    ProjectLoader projectLoader(FilePath);
    projectLoader.Load(FilePath + "trijumpSD_shortcut.xml");
    vector<Frame*> frames = projectLoader.getFrames();

    //Copy image and skeleton from first keyframe
    int FirstKeyframe = 0;
    Mat image = frames[FirstKeyframe]->getImage();
    Mat mask = frames[FirstKeyframe]->getMask();
    Skeleton skeleton = frames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    //Select body part for testing
    const int  partID = 6;
    //Copy body part
    BodyPart bodyPart = *skeleton.getBodyPart(partID);
    //Copy part joints 
    BodyJoint* j0 = skeleton.getBodyJoint(bodyPart.getParentJoint());
    BodyJoint* j1 = skeleton.getBodyJoint(bodyPart.getChildJoint());
    //Copy joints location
    Point2f p0 = j0->getImageLocation();
    Point2f p1 = j1->getImageLocation();

    //Calculate part models descriptors 
    uint32_t minHessian = 500;
    SurfDetector D;
    map <uint32_t, SurfDetector::PartModel> PartModels = D.computeDescriptors(frames[FirstKeyframe], minHessian);

    //Copy ideal part model for selected body part
    SurfDetector::PartModel partModel0 = PartModels[partID];

    //Create a bad part models for selected body part
    SurfFeatureDetector D1(minHessian);
    vector <KeyPoint> _keyPoints;
    D1.detect(image, _keyPoints);

    Point2f shift1(100, 40), shift2(-100, 0);
    float boneLength = D.getBoneLength(p0, p1);
    float boneWidth = D.getBoneWidth(boneLength, bodyPart);
    SurfDetector::PartModel partModel1, partModel2;
    partModel1.partModelRect = D.getBodyPartRect(bodyPart, p0 + shift1, p1 + shift1, Size(boneLength, boneWidth));
    partModel2.partModelRect = D.getBodyPartRect(bodyPart, p0 + shift2, p1 + shift2, Size(boneLength, boneWidth));

    for (int i = 0; i < _keyPoints.size(); i++)
    {
      if (partModel1.partModelRect.containsPoint(_keyPoints[i].pt) > 0)
        partModel1.keyPoints.push_back(_keyPoints[i]);
      if (partModel2.partModelRect.containsPoint(_keyPoints[i].pt) > 0)
        partModel2.keyPoints.push_back(_keyPoints[i]);
    }

    SurfDescriptorExtractor extractor;
    extractor.compute(image, partModel1.keyPoints, partModel1.descriptors);
    extractor.compute(image, partModel2.keyPoints, partModel2.descriptors);

    //Put expected and actual polygons and keypoints into image file
    Mat KeyPoints_image, PartKeyPoints;

    drawKeypoints(image, _keyPoints, KeyPoints_image, 128);
    PutPartRect(KeyPoints_image, partModel2.partModelRect.asVector(), Scalar(50, 50, 255));
    PutPartRect(KeyPoints_image, partModel1.partModelRect.asVector(), Scalar(255, 50, 50));
    PutPartRect(KeyPoints_image, partModel0.partModelRect.asVector(), Scalar(50, 255, 50));
    imwrite("SurfDetector_CompareTest.jpg", KeyPoints_image);
    KeyPoints_image.release();

    //Run "Train"
    map <string, float> params;
    D.train(frames, params);

    //Run "compare" function
    float score0 = D.compare(bodyPart, partModel0, p0, p1);
    float score1 = D.compare(bodyPart, partModel1, p0 + shift1, p1 + shift1);
    float score2 = D.compare(bodyPart, partModel2, p0 + shift2, p1 + shift2);
    
    //Put results
    cout << "\npartModel0.keyPoints.size = " << partModel0.keyPoints.size() << endl;
    cout <<  "partModel1.keyPoints.size = " << partModel1.keyPoints.size() << endl;
    cout <<  "partModel2.keyPoints.size = " << partModel1.keyPoints.size() << endl << endl;

    cout << "score0 = " << score0 << endl;
    cout << "score1 = " << score1 << endl;
    cout << "score2 = " << score2 << endl << endl;

    //Compare scores
    EXPECT_NE(score0, score1) << "Must be different values" << endl; // ???
    EXPECT_NEAR(1, abs(score0 - score2), 0.3) << "Must be near to 1" << endl; // ???
    cout << "See part rects locations in the file: SurfDetector_CompareTest.jpg" << endl << endl;
}