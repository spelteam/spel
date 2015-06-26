#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

#include <gtest/gtest.h>
#include <detector.hpp>
#include <SurfDetector.hpp>
#include "projectLoader.hpp"
#include "limbLabel.hpp"

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
    SurfFeatureDetector D1(minHessian);
    //Frame keypoints
    vector <KeyPoint> expected_FrameKeyPoints; 
    D1.detect(image, expected_FrameKeyPoints);
    //Part keypoints
    vector <KeyPoint> expected_PartKeyPoints;
    for (int i = 0; i < expected_FrameKeyPoints.size(); i++)
      if (rect.containsPoint(expected_FrameKeyPoints[i].pt) > 0)
        expected_PartKeyPoints.push_back(expected_FrameKeyPoints[i]);
    //Part descriptors
    Mat expected_PartDescriptors;
    SurfDescriptorExtractor extractor;
    extractor.compute(image, expected_PartKeyPoints, expected_PartDescriptors);

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
    vector<Point2f> actual_PartPolygon = actual_PartModel.partModelRect.asVector();
    actual_PartPolygon.push_back(actual_PartPolygon[0]);
    for (int i = 1; i < actual_PartPolygon.size(); i++)
        line(image, actual_PartPolygon[i - 1], actual_PartPolygon[i], 255, 1, 1);

    vector<Point2f> expected_PartPolygon = rect.asVector();
    actual_PartPolygon.push_back(expected_PartPolygon[0]);
    for (int i = 1; i < expected_PartPolygon.size(); i++)
        line(image, expected_PartPolygon[i - 1], expected_PartPolygon[i], 100, 1, 1);

    Mat KeyPoints_image, PartKeyPoints, expected_KeyPoints;
    drawKeypoints(image, expected_FrameKeyPoints, KeyPoints_image, 128);
    imwrite("KeyPoints_Frame.jpg", KeyPoints_image);
    drawKeypoints(image, actual_PartModels[partID].keyPoints, PartKeyPoints, 128);
    imwrite("KeyPoints_CurrentPart.jpg", PartKeyPoints);
    drawKeypoints(image, expected_PartKeyPoints, expected_KeyPoints, 128);
    imwrite("KeyPoints_Expected.jpg", expected_KeyPoints);

    cout << "See keypoints in files: KeyPoints_Frame.jpg, KeyPoints_CurrentPart.jpg, KeyPoints_Expected.jpg" << endl;

    KeyPoints_image.release();
    PartKeyPoints.release();
}
