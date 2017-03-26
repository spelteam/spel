// SPEL definitions
#include "predef.hpp"

#if defined(WINDOWS) && defined(_MSC_VER)
#include <Windows.h>
#endif

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

#include <gtest/gtest.h>
#include <detector.hpp>
#include <surfDetector.hpp>
#include "limbLabel.hpp"
#include "TestsFunctions.hpp"
#include "spelGeometry.hpp"

using namespace std;
#if defined (HAVE_OPENCV_XFEATURES2D)
using namespace xfeatures2d;
#endif

namespace SPEL
{
  //vector<Frame*> SFrames;

  vector <KeyPoint> SelectKeypoints(vector <KeyPoint> FrameKeypoints, spelRECT<Point2f> rect)
  {
    vector <KeyPoint> keyPoints;
    for (unsigned int i = 0; i < FrameKeypoints.size(); i++)
      if (rect.containsPoint(FrameKeypoints[i].pt) > 0)
        keyPoints.push_back(FrameKeypoints[i]);
    return keyPoints;
  }

  TEST(surfDetectorTests, Constructor)
  {
    uint32_t id = 0x53440000;

    SurfDetector d;
    EXPECT_EQ(id, d.getID());
  }

  //Testing "computeDescriptors" functions
  TEST(surfDetectorTests, computeDescriptors)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/A/", "trijumpSD_shortcut.xml");
    vector<Frame*> SFrames = project.getFrames();

    //Counting a keyframes
    int FirstKeyframe = FirstKeyFrameNum(SFrames);

    //Copy image and skeleton from first keyframe
    Mat image = SFrames[FirstKeyframe]->getImage();
    Mat mask = SFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = SFrames[FirstKeyframe]->getSkeleton();
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
    map <uint32_t, SurfDetector::PartModel> actual_PartModels = D.computeDescriptors(SFrames[FirstKeyframe], minHessian);
    SurfDetector::PartModel actual_PartModel = D.computeDescriptors(bodyPart, p0, p1, image, actual_PartModels[partID].keyPoints);

    //Calculate expected values

    // Part rect
    float boneLength = BodyPart::getBoneLength(p0, p1);
    float boneWidth = bodyPart.getBoneWidth(boneLength);
    spelRECT <Point2f> rect = bodyPart.getBodyPartRect(p0, p1, Size(static_cast <int> (boneLength), static_cast <int> (boneWidth)));
    //Frame keypoints
    vector <KeyPoint> expected_FrameKeyPoints;
    Ptr <SurfFeatureDetector> D1 = SurfFeatureDetector::create(minHessian);
    D1->detect(image, expected_FrameKeyPoints);
    //Part keypoints
    vector <KeyPoint> expected_PartKeyPoints = SelectKeypoints(expected_FrameKeyPoints, rect);

    //Part descriptors
    Mat expected_PartDescriptors;
    Ptr <SurfDescriptorExtractor> extractor = SurfDescriptorExtractor::create();
    extractor->compute(image, expected_PartKeyPoints, expected_PartDescriptors);
    //Compare

    //Compare part rect
    EXPECT_EQ(rect.asVector(), actual_PartModels[partID].partModelRect.asVector());

    //Compare keypoints size
    EXPECT_EQ(expected_PartKeyPoints.size(), actual_PartModels[partID].keyPoints.size());

    //Compare current part keypoints
    for (unsigned int i = 0; i < min(expected_PartKeyPoints.size(), actual_PartModels[partID].keyPoints.size()); i++)
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

    image.release();
    mask.release();
    
    //project.TestProjectLoader::~TestProjectLoader();
    SFrames.clear();
  }


  TEST(surfDetectorTests, train)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/A/", "trijumpSD_shortcut.xml");
    vector<Frame*> SFrames = project.getFrames();

    //Counting a keyframes
    int FirstKeyframe = FirstKeyFrameNum(SFrames);

    //Copy image and skeleton from first keyframe
    Mat image = SFrames[FirstKeyframe]->getImage();
    Mat mask = SFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = SFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();
    Size ImageSize = image.size();

    //Calculate actual value
    uint32_t minHessian = 500;
    SurfDetector D;
    map <string, float> params;
    params.emplace(pair<string, float>("maxFrameHeight", ImageSize.height));
    D.train(SFrames, params);

    //Calculate expected value
    vector <KeyPoint> keyPoints;
    Ptr <SurfFeatureDetector> D1 = SurfFeatureDetector::create(minHessian);
    D1->detect(image, keyPoints);
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
      float boneLength = BodyPart::getBoneLength(p0, p1);
      float boneWidth = (*part).getBoneWidth(boneLength);
      spelRECT<Point2f>  rect = (*part).getBodyPartRect(p0, p1, Size(static_cast <int> (boneLength), static_cast <int> (boneWidth)));
      expected_rects[partID] = rect.asVector();

      for (unsigned int i = 0; i < keyPoints.size(); i++)
        if (rect.containsPoint(keyPoints[i].pt) > 0)
          temp.push_back(keyPoints[i]);
      expected_Keypoints.emplace(pair<uint32_t, vector<KeyPoint>>(partID, temp));

      Mat temp_descriptors;
      Ptr <SurfDescriptorExtractor> extractor = SurfDescriptorExtractor::create();
      extractor->compute(image, expected_Keypoints[partID], temp_descriptors);
      expected_descriptors.emplace(pair<uint32_t, Mat>(partID, temp_descriptors.clone()));
      temp_descriptors.release();
      temp.clear();
    }

    //Compare
    vector<Point2f> empty_rect = { Point2f(0, 0), Point2f(0, 0), Point2f(0, 0), Point2f(0, 0) };
    for (unsigned int f = 0; f < SFrames.size(); f++)
      for (tree<BodyPart>::iterator p = partTree.begin(); p != partTree.end(); p++)
      {
        int partID = p->getPartID();
        if (SFrames[f]->getFrametype() != INTERPOLATIONFRAME)
        {
          EXPECT_EQ(expected_rects[partID], D.partModels[f][partID].partModelRect.asVector());
          for (unsigned int i = 0; i < expected_Keypoints[partID].size(); i++)
            EXPECT_EQ(expected_Keypoints[partID][i].pt, D.partModels[f][partID].keyPoints[i].pt);
          bool All_values_equal = true;
          Size size = expected_descriptors[partID].size();
          for (int i = 0; i < size.height; i++)
            for (int k = 0; k < size.width; k++)
              All_values_equal = expected_descriptors[partID].at<float>(i, k) == D.partModels[f][partID].descriptors.at<float>(i, k);
          EXPECT_TRUE(All_values_equal);
        }
        if (SFrames[f]->getFrametype() == INTERPOLATIONFRAME)
        {
          EXPECT_EQ(empty_rect, D.partModels[f][partID].partModelRect.asVector()); //?
          EXPECT_TRUE(D.partModels[f][partID].keyPoints.empty());
          for (unsigned int i = 0; i < D.partModels[f][partID].keyPoints.size(); i++)
            cout << D.partModels[f][partID].keyPoints[i].pt << endl;
        }
        expected_Keypoints[partID].clear();
      }

    expected_rects.clear();
    
    image.release();
    mask.release();
    //project.TestProjectLoader::~TestProjectLoader();
    SFrames.clear();
  }

  //Temporary test
  TEST(surfDetectorTests, compare)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/A/", "trijumpSD_shortcut.xml");
    vector<Frame*> SFrames = project.getFrames();

    //Copy image and skeleton from first keyframe
    int FirstKeyframe = 0;
    Mat image = SFrames[FirstKeyframe]->getImage();
    Mat mask = SFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = SFrames[FirstKeyframe]->getSkeleton();
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
    map <uint32_t, SurfDetector::PartModel> PartModels = D.computeDescriptors(SFrames[FirstKeyframe], minHessian);

    //Copy ideal part model for selected body part
    SurfDetector::PartModel partModel0 = PartModels[partID];

    //Create a bad part models for selected body part
    vector <KeyPoint> _keyPoints;
    Ptr <SurfFeatureDetector> D1 = SurfFeatureDetector::create(minHessian);
    D1->detect(image, _keyPoints);

    const float knnMatchKoeff = 0.8f;

    Point2f shift1(100, 40), shift2(-100, 0);
    float boneLength = BodyPart::getBoneLength(p0, p1);
    float boneWidth = bodyPart.getBoneWidth(boneLength);
    SurfDetector::PartModel partModel1, partModel2;
    partModel1.partModelRect = bodyPart.getBodyPartRect(p0 + shift1, p1 + shift1, Size(static_cast <int> (boneLength), static_cast <int> (boneWidth)));
    partModel2.partModelRect = bodyPart.getBodyPartRect(p0 + shift2, p1 + shift2, Size(static_cast <int> (boneLength), static_cast <int> (boneWidth)));

    for (unsigned int i = 0; i < _keyPoints.size(); i++)
    {
      if (partModel1.partModelRect.containsPoint(_keyPoints[i].pt) > 0)
        partModel1.keyPoints.push_back(_keyPoints[i]);
      if (partModel2.partModelRect.containsPoint(_keyPoints[i].pt) > 0)
        partModel2.keyPoints.push_back(_keyPoints[i]);
    }
    Ptr <SurfDescriptorExtractor> extractor = SurfDescriptorExtractor::create();
    extractor->compute(image, partModel1.keyPoints, partModel1.descriptors);
    extractor->compute(image, partModel2.keyPoints, partModel2.descriptors);
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
    D.train(SFrames, params);

    //Run "compare" function
    float score0 = D.compare(bodyPart, partModel0, p0, p1, knnMatchKoeff);
    float score1 = D.compare(bodyPart, partModel1, p0 + shift1, p1 + shift1, knnMatchKoeff);
    float score2 = D.compare(bodyPart, partModel2, p0 + shift2, p1 + shift2, knnMatchKoeff);

    //Put results
    cout << "\npartModel0.keyPoints.size = " << partModel0.keyPoints.size() << endl;
    cout << "partModel1.keyPoints.size = " << partModel1.keyPoints.size() << endl;
    cout << "partModel2.keyPoints.size = " << partModel1.keyPoints.size() << endl << endl;

    cout << "score0 = " << score0 << endl;
    cout << "score1 = " << score1 << endl;
    cout << "score2 = " << score2 << endl << endl;

    //Compare scores
    EXPECT_NE(score0, score1) << "Must be different values" << endl; // ???
    EXPECT_LT(score0, score1) << "Score0 is score for ideal part model" << endl;
    EXPECT_LT(score1, abs(score2)) << "Score2 is score for bad part model" << endl;
    //EXPECT_NEAR(1, abs(score0 - score2), 0.3) << "Must be near to 1" << endl; // ???
    cout << "See part rects locations in the file: SurfDetector_CompareTest.jpg" << endl << endl;
    
    image.release();
    mask.release();
    //project.TestProjectLoader::~TestProjectLoader();
    SFrames.clear();
  }

  TEST(surfDetectorTests, generateLabel)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/A/", "trijumpSD_shortcut.xml");
    vector<Frame*> SFrames = project.getFrames();

    //Copy image and skeleton from first keyframe
    int FirstKeyframe = 0;
    Mat image = SFrames[FirstKeyframe]->getImage();
    Mat mask = SFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = SFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    //Select body part for testing
    int  partID = 6;
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
    const float knnMatchCoeff = 0.8f;
    SurfDetector D;
    map <uint32_t, SurfDetector::PartModel> PartModels = D.computeDescriptors(SFrames[FirstKeyframe], minHessian);

    //Create part model for selected LimbLabel {p0, p1}
    vector <KeyPoint> _keyPoints;
    Ptr <SurfFeatureDetector> D1 = SurfFeatureDetector::create(minHessian);
    D1->detect(image, _keyPoints);
    float boneLength = BodyPart::getBoneLength(p0, p1);
    float boneWidth = bodyPart.getBoneWidth(boneLength);
    SurfDetector::PartModel partModel1;
    partModel1.partModelRect = bodyPart.getBodyPartRect(p0, p1, Size(static_cast <int> (boneLength), static_cast <int> (boneWidth)));
    for (unsigned int i = 0; i < _keyPoints.size(); i++)
    {
     if (partModel1.partModelRect.containsPoint(_keyPoints[i].pt) > 0)
       partModel1.keyPoints.push_back(_keyPoints[i]);
    }
     Ptr <SurfDescriptorExtractor> extractor = SurfDescriptorExtractor::create();
     extractor->compute(image, partModel1.keyPoints, partModel1.descriptors);

    vector<Score> scores;
    Score score(D.compare(bodyPart, partModel1, p0, p1, knnMatchCoeff), std::to_string(D.getID()));
    scores.push_back(score);
    float rot = float(spelHelper::angle2D(1.0f, 0.0f, p1.x - p0.x, p1.y - p0.y) * (180.0 / M_PI));
    LimbLabel expected_Label(partID, 0.5*(p0 + p1), rot, BuildPartRect(j0, j1, bodyPart.getLWRatio()).asVector(), scores, false);
    scores.clear();

    map <string, float> detectParams;
    auto detectorHelper = new SurfDetectorHelper();
    detectorHelper->keyPoints = partModel1.keyPoints;

    //Run "GenerateLabel"
    LimbLabel actual_Label = D.generateLabel(bodyPart, SFrames[FirstKeyframe], p0, p1, detectorHelper, detectParams);

    delete detectorHelper;

    //Compare
    EXPECT_FLOAT_EQ(expected_Label.getAngle(), actual_Label.getAngle());
    EXPECT_EQ(expected_Label.getCenter(), actual_Label.getCenter());
    EXPECT_EQ(expected_Label.getLimbID(), actual_Label.getLimbID());
    EXPECT_EQ(expected_Label.getScores(), actual_Label.getScores());
    unsigned int n = expected_Label.getPolygon().size();
    for (unsigned int i = 0; i < n; i++)
      EXPECT_EQ(static_cast<Point2i>(expected_Label.getPolygon()[i]), static_cast<Point2i>(actual_Label.getPolygon()[i]));

    //Checking SurfDetector.LabelModels
   /*int n = D.labelModels[SFrames[FirstKeyframe]->getID()][bodyPart.getPartID()].size()-1;
    //D.labelModels[CurrentFrameID][CurrentPartID][CurrentLimbLabel][CurrentLimbLabel] is empty?
    SurfDetector::PartModel labelModels_partID = D.labelModels[SFrames[FirstKeyframe]->getID()][bodyPart.getPartID()][n];
    EXPECT_EQ(partModel1.partModelRect.asVector(), labelModels_partID.partModelRect.asVector());
    EXPECT_EQ(partModel1.keyPoints.size(), labelModels_partID.keyPoints.size());
    EXPECT_EQ(partModel1.descriptors.size(), labelModels_partID.descriptors.size());*/
     
    image.release();
    mask.release();
    //project.TestProjectLoader::~TestProjectLoader();
    SFrames.clear();
  }

TEST(surfDetectorTests, detect)
  {   
    //Load the input data
    //TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/A/", "trijumpSD_shortcut.xml");
    TestProjectLoader project("speltests_TestData/SurfDetectorTestsData/C/", "skier.xml");
    vector<Frame*> SFrames = project.getFrames();

    TestProjectLoader project_pattern("speltests_TestData/SurfDetectorTestsData/C/", "skier_pattern.xml");
    vector<Frame*> Pattern = project_pattern.getFrames();
    Skeleton SkeletonPattern = Pattern[1]->getSkeleton();
   
    // Run "detect"
    SurfDetector D;
    map <string, float> params;
    long train_t0 = clock();
    D.train(SFrames, params);
    long train_t1 = clock();
    ASSERT_GT(D.getPartModels().size(), 0);
 
    // Run "train"
    SFrames[1]->setSkeleton(SkeletonPattern);
    params.emplace(pair<string, float>("knnMathCoeff", 0.7f));
    map<uint32_t, vector<LimbLabel>> Labels;
    long decect_t0 = clock();
    Labels = D.detect(SFrames[1], params, Labels);
    long decect_t1 = clock();
    ASSERT_GT(Labels.size(), 0);

    // Set test parameters
    int TopLabelsCount = 4; // Size of "labels top list"
    float TolerableLinearError = 30; // Linear error in pixels
                                                                                     
    // Compare limb labels with ideal bodyparts from keyframe 
    map<int, vector<float>> LinearErrors = LabelsLinearErrors(Labels, SkeletonPattern);
    map<int, vector<float>> AngleErrors = LabelsAngleErrors(Labels, SkeletonPattern);

    // Write debug information into text file
    string OutputFileName = "DetectTest_SURF.txt";
    ofstream fout(OutputFileName);
    fout << "\n  DETECT RESULTS\n\n";
    PutSigificantErrors(fout, LinearErrors, AngleErrors, TopLabelsCount);
    PutLabels(fout, "", Labels, LinearErrors, AngleErrors, TopLabelsCount);
    PutLabels(fout, "", Labels, LinearErrors, AngleErrors, 0);
    fout.close();

    // Cheking the values and put test result
    vector<int> notFoundedParts = selectNotFoundedParts(SkeletonPattern, LinearErrors, Labels, TolerableLinearError, TopLabelsCount);
    cout << "\n EXECUTION TIME\n" << "\nTrain: " << clock_to_ms(train_t1 - train_t0) << " ms\n";
    cout << "Detect: " << clock_to_ms(decect_t1 - decect_t0) << " ms\n";

    cout << "\n TEST PARAMETERS\n\n";
    cout << "Image size = " << Pattern[0]->getMask().size() << endl;
    cout << "TolerableLinearError = " << TolerableLinearError << endl;
    cout << "TopLabelsCount = " << TopLabelsCount << endl;

    cout << "\n DETECT RESULTS\n\n";
    PutSigificantErrors(cout, LinearErrors, AngleErrors, TopLabelsCount);

    int FoundedPartsCount = Labels.size() - notFoundedParts.size();
    int PartsCount = SkeletonPattern.getPartTree().size();
    EXPECT_EQ(PartsCount, FoundedPartsCount);

    if (notFoundedParts.size() > 0)
    {
      cout << "Body parts with id: " << notFoundedParts[0];
      for (unsigned int i = 1; i < notFoundedParts.size(); i++)
        cout << ", " << notFoundedParts[i];
      cout << " - does not have effective labels in the top of labels list." << endl;
    }
    cout << "\nLimbLabels saved in file: " << OutputFileName << endl << endl;

    //project.TestProjectLoader::~TestProjectLoader();
    SFrames.clear();
  }

  TEST(surfDetectorTests, PartModel)
  {
    // Default partModel
    SurfDetector::PartModel partModel;

    EXPECT_EQ(0, static_cast<int>(partModel.keyPoints.size()));
    EXPECT_EQ(Size(0, 0), partModel.descriptors.size());
    Point2f X(0.0f, 0.0f);
    EXPECT_EQ(spelRECT<cv::Point2f>(X, X, X, X), partModel.partModelRect);

    // Assigned partModel
    Point2f z(1.0f, 2.0f);
    spelRECT<cv::Point2f> rect(X, X, X, X);

    vector <cv::KeyPoint> KeyPoints;
    cv::KeyPoint abstract_keypoint;
    abstract_keypoint.pt = Point2f(0.1f, 0.2f);
    abstract_keypoint.size = 1;
    abstract_keypoint.angle = 0.3f;
    abstract_keypoint.response = 0.4f;
    abstract_keypoint.octave = 0; 
    abstract_keypoint.class_id = 1;
    KeyPoints.push_back(abstract_keypoint);

    cv::Mat abstract_descriptors = Mat(8, 8, cv::DataType<float>::type, static_cast<float>(1.1f));

    partModel.partModelRect = rect;
    partModel.keyPoints = KeyPoints;
    partModel.descriptors = abstract_descriptors;

    EXPECT_EQ(rect, partModel.partModelRect);
    EXPECT_EQ(KeyPoints.size(), partModel.keyPoints.size());
    EXPECT_EQ(abstract_keypoint.pt, partModel.keyPoints[0].pt);
    EXPECT_EQ(abstract_keypoint.size, partModel.keyPoints[0].size);
    EXPECT_FLOAT_EQ(abstract_keypoint.angle, partModel.keyPoints[0].angle);
    EXPECT_FLOAT_EQ(abstract_keypoint.response, partModel.keyPoints[0].response);
    EXPECT_EQ(abstract_keypoint.octave, partModel.keyPoints[0].octave);
    EXPECT_EQ(abstract_keypoint.class_id, partModel.keyPoints[0].class_id);
    EXPECT_EQ(abstract_descriptors.size(), partModel.descriptors.size());
    EXPECT_FLOAT_EQ(abstract_descriptors.at<float>(0, 0), partModel.descriptors.at<float>(0,0));
    
    // Copyed partModel
    SurfDetector::PartModel partModel1 = partModel;	

    EXPECT_EQ(rect, partModel1.partModelRect);
    EXPECT_EQ(KeyPoints.size(), partModel1.keyPoints.size());
    EXPECT_EQ(abstract_keypoint.pt, partModel1.keyPoints[0].pt);
    EXPECT_EQ(abstract_keypoint.size, partModel1.keyPoints[0].size);
    EXPECT_FLOAT_EQ(abstract_keypoint.angle, partModel1.keyPoints[0].angle);
    EXPECT_FLOAT_EQ(abstract_keypoint.response, partModel1.keyPoints[0].response);
    EXPECT_EQ(abstract_keypoint.octave, partModel1.keyPoints[0].octave);
    EXPECT_EQ(abstract_keypoint.class_id, partModel1.keyPoints[0].class_id);
    EXPECT_EQ(abstract_descriptors.size(), partModel1.descriptors.size());
    EXPECT_FLOAT_EQ(abstract_descriptors.at<float>(0, 0), partModel1.descriptors.at<float>(0, 0));
  }

  TEST(surfDetectorTests, getPartModels)
  {
    // Prepare test data
    SurfDetector detector;
    for (uint32_t i = 0; i < 3; i++)
    {
      std::map <uint32_t, SurfDetector::PartModel> temp;
      for (uint32_t k = 0; k < 3; k++)
      {
        // Assigned partModel
        SurfDetector::PartModel partModel;
        Point2f X(k, k);
        spelRECT<cv::Point2f> rect(X, X, X, X);

        vector <cv::KeyPoint> KeyPoints;
        cv::KeyPoint abstract_keypoint;
        abstract_keypoint.pt = Point2f(k, k);
        abstract_keypoint.size = k;
        abstract_keypoint.angle = k;
        abstract_keypoint.response = k;
        abstract_keypoint.octave = k;
        abstract_keypoint.class_id = k;
        KeyPoints.push_back(abstract_keypoint);

        cv::Mat abstract_descriptors = Mat(8, 8, cv::DataType<float>::type, static_cast<float>(k));

        partModel.partModelRect = rect;
        partModel.keyPoints = KeyPoints;
        partModel.descriptors = abstract_descriptors;
        temp.emplace(pair<uint32_t, SurfDetector::PartModel>(k, partModel));
      }
      detector.partModels.emplace(pair<uint32_t, std::map <uint32_t, SurfDetector::PartModel>>(i, temp));
      temp.clear();
    }

    for (uint32_t i = 0; i < 3; i++)
    {
      std::map <uint32_t, SurfDetector::PartModel> temp;
      for (uint32_t k = 0; k < 3; k++)
      {
        // Assigned partModel
        SurfDetector::PartModel partModel;
        Point2f X(k, k);
        spelRECT<cv::Point2f> rect(X, X, X, X);

        vector <cv::KeyPoint> KeyPoints;
        cv::KeyPoint abstract_keypoint;
        abstract_keypoint.pt = Point2f(k, k);
        abstract_keypoint.size = k;
        abstract_keypoint.angle = k;
        abstract_keypoint.response = k;
        abstract_keypoint.octave = k;
        abstract_keypoint.class_id = k;
        KeyPoints.push_back(abstract_keypoint);

        cv::Mat abstract_descriptors = Mat(8, 8, cv::DataType<float>::type, static_cast<float>(k));

        EXPECT_EQ(rect, detector.partModels[i][k].partModelRect);
        EXPECT_EQ(KeyPoints.size(), detector.partModels[i][k].keyPoints.size());
        EXPECT_EQ(abstract_keypoint.pt, detector.partModels[i][k].keyPoints[0].pt);
        EXPECT_EQ(abstract_keypoint.size, detector.partModels[i][k].keyPoints[0].size);
        EXPECT_FLOAT_EQ(abstract_keypoint.angle, detector.partModels[i][k].keyPoints[0].angle);
        EXPECT_FLOAT_EQ(abstract_keypoint.response, detector.partModels[i][k].keyPoints[0].response);
        EXPECT_EQ(abstract_keypoint.octave, detector.partModels[i][k].keyPoints[0].octave);
        EXPECT_EQ(abstract_keypoint.class_id, detector.partModels[i][k].keyPoints[0].class_id);
        EXPECT_EQ(abstract_descriptors.size(), detector.partModels[i][k].descriptors.size());
        EXPECT_FLOAT_EQ(abstract_descriptors.at<float>(0, 0), detector.partModels[i][k].descriptors.at<float>(0, 0));
      }
      detector.partModels[i].clear();
    }
    detector.partModels.clear();
  }

}
