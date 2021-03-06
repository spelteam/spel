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
#include <hogDetector.hpp>
#include "projectLoader.hpp"
#include "limbLabel.hpp"
#include "spelHelper.hpp"
#include "TestsFunctions.hpp"
#include "spelGeometry.hpp"

namespace SPEL
{
  /*string HOGDetectorTestProjectPath = "speltests_TestData/CHDTrainTestData/";
  string HOGDetectorTestProjectPName = "trijumpSD_50x41.xml";
  vector<Frame*> HFrames;
  HogDetector D;*/

  vector <vector <vector <float>>> decodeDescriptor(vector<float> descriptors, Size wndSize, Size blockSize, Size blockStride, Size cellSize, int nbins)
  {
    vector <vector <vector <float>>> gradientStrengths;
    vector <vector <uint32_t>> counter;

    for (int i = 0; i < wndSize.height; i += cellSize.height)
    {
      gradientStrengths.push_back(vector <vector <float>>());
      counter.push_back(vector <uint32_t>());
      for (int j = 0; j < wndSize.width; j += cellSize.width)
      {
        gradientStrengths.at(i / cellSize.height).push_back(vector <float>());
        counter.at(i / cellSize.height).push_back(0);
        for (int b = 0; b < nbins; b++)
          gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).push_back(0.0f);
      }
    }

    int d = 0;
    for (int n = 0; n + blockStride.height < wndSize.height; n += blockStride.height)// window rows
      for (int k = 0; k + blockStride.width < wndSize.width; k += blockStride.width)// window cols				
        for (int r = n; r < n + blockSize.height; r += cellSize.height)// block rows					
          for (int c = k; c < k + blockSize.width; c += cellSize.width)// block cols						
            for (int b = 0; b < nbins; b++)// nbins
            {
              gradientStrengths.at(r / cellSize.height).at(c / cellSize.width).at(b) += descriptors.at(d);
              if (b == 0) counter.at(r / cellSize.height).at(c / cellSize.width)++;
              d++;
            }

    for (int i = 0; i < wndSize.height; i += cellSize.height)
      for (int j = 0; j < wndSize.width; j += cellSize.width)
        for (int b = 0; b < nbins; b++)
          if (counter.at(i / cellSize.height).at(j / cellSize.width) == 0)
            gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).at(b) = 0;
          else
            gradientStrengths.at(i / cellSize.height).at(j / cellSize.width).at(b) /= counter.at(i / cellSize.height).at(j / cellSize.width);

    return gradientStrengths;
  }

  map<int, Point2f> getImageLocations(Skeleton skeleton)
  {
    map<int, Point2f> Locations;
    tree <BodyJoint> jointTree = skeleton.getJointTree();
    for (tree <BodyJoint>::iterator i = jointTree.begin(); i != jointTree.end(); ++i)
      Locations.emplace(pair<int, Point2f>(i->getLimbID(), i->getImageLocation()));
    return Locations;
  }

  class _LimbIDCompare
  {
  public:
    bool operator () (vector<LimbLabel> X, vector<LimbLabel> Y)
    {
      return Y[0].getLimbID() > X[0].getLimbID();
    }
  };

  // Selecting locations of all body part from skeleton
  map<int, pair<Point2f, Point2f>> _getPartLocations(Skeleton skeleton)
  {
    map<int, pair<Point2f, Point2f>> PartLocations;
    BodyJoint* J0, *J1;
    Point2f p0, p1;
    tree <BodyPart> partTree = skeleton.getPartTree();
    for (tree <BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
    {
      J0 = skeleton.getBodyJoint(i->getChildJoint());
      J1 = skeleton.getBodyJoint(i->getParentJoint());
      p0 = J0->getImageLocation();
      p1 = J1->getImageLocation();
      PartLocations.emplace(pair<int, pair<Point2f, Point2f>>(i->getPartID(), pair<Point2f, Point2f>(p0, p1)));
    }
    return PartLocations;
  }

  TEST(HOGDetectorTests, computeDescriptor)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> HFrames = project.getFrames();

    //Counting a keyframes
    //int KeyframesCount = 0;
    int FirstKeyframe = 0;

    //Copy image and skeleton from first keyframe
    Mat image = HFrames[FirstKeyframe]->getImage();
    Mat mask = HFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = HFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    //Select body part for testing
    const int  partID = 6;
    //Copy body part
    BodyPart bodyPart = *skeleton.getBodyPart(partID);
    //Copy part joints 
    BodyJoint* j0 = skeleton.getBodyJoint(bodyPart.getParentJoint());
    BodyJoint* j1 = skeleton.getBodyJoint(bodyPart.getChildJoint());

    // Set descriptor parameters
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    Size wndSize = Size(64, 128);
    const int nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;
    bool bGrayImages = false;

    //Calculate actual value
    HogDetector D(nbins, wndSize, padding, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType);
    HogDetector::PartModel partModel;
    partModel = D.computeDescriptors(bodyPart, j0->getImageLocation(), j1->getImageLocation(), image, wndSize);

    //Calculate expected value
    vector<float> descriptorsValues;
    vector<Point> locations;
    HOGDescriptor d(wndSize, blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);
    d.compute(partModel.partImage, descriptorsValues);

    // Compare
    EXPECT_EQ(descriptorsValues.size(), partModel.descriptors.size());
    for (unsigned int i = 0; i < descriptorsValues.size(); i++)
      EXPECT_EQ(descriptorsValues[i], partModel.descriptors[i]);

    vector <vector <vector <float>>> G = decodeDescriptor(descriptorsValues, wndSize, blockSize, blockStride, cellSize, nbins);
    for (unsigned int i = 0; i < G.size(); i++)
      for (unsigned int k = 0; k < G[i].size(); k++)
        for (unsigned int n = 0; n < G[i][k].size(); n++)
          EXPECT_EQ(G[i][k][n], partModel.gradientStrengths[i][k][n]);

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    HFrames.clear();
  }

  TEST(HOGDetectorTests, computeDescriptors)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> HFrames = project.getFrames();

    //Counting a keyframes
    //int KeyframesCount = 0;
    int FirstKeyframe = 0;

    //Copy image and skeleton from first keyframe
    Mat image = HFrames[FirstKeyframe]->getImage();
    Mat mask = HFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = HFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    // Set descriptor parameters
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    Size wndSize = Size(64, 128);
    const int nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;
    bool bGrayImages = false;

    //Calculate actual value
    HogDetector D(nbins, wndSize, padding, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType);
    D.m_frames = HFrames;
    D.m_partSize = D.getMaxBodyPartHeightWidth(blockSize, 1.0f);
    map <uint32_t, HogDetector::PartModel> partModels;
    partModels = D.computeDescriptors(HFrames[FirstKeyframe]);

    //Calculate expected value
    vector<vector<float>> allDescriptors;
    for (unsigned int partID = 0; partID < partTree.size(); partID++)
    {
      vector<float> descriptorsValues;
      BodyPart bodyPart = *skeleton.getBodyPart(partID);//Copy body part	
      //BodyJoint* j0 = skeleton.getBodyJoint(bodyPart.getParentJoint());//Copy part joints 
      //BodyJoint* j1 = skeleton.getBodyJoint(bodyPart.getChildJoint());
      HOGDescriptor d(D.m_partSize[partID], blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);
      d.compute(partModels[partID].partImage, descriptorsValues);
      allDescriptors.push_back(descriptorsValues);
    }

    //Compare		
    Skeleton S = HFrames[FirstKeyframe]->getSkeleton();
    for (unsigned int partID = 0; partID < partTree.size(); partID++)
    {
      EXPECT_EQ(allDescriptors[partID].size(), partModels[partID].descriptors.size());
      for (unsigned int i = 0; i < allDescriptors[partID].size(); i++)
        EXPECT_EQ(allDescriptors[partID][i], partModels[partID].descriptors[i]);

      vector <vector <vector <float>>> G = decodeDescriptor(allDescriptors[partID], D.m_partSize[partID], blockSize, blockStride, cellSize, nbins);
      for (unsigned int i = 0; i < G.size(); i++)
        for (unsigned int k = 0; k < G[i].size(); k++)
          for (unsigned int n = 0; n < G[i][k].size(); n++)
            EXPECT_EQ(G[i][k][n], partModels[partID].gradientStrengths[i][k][n]);

      allDescriptors[partID].clear();

    }
    allDescriptors.clear();

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    HFrames.clear();
  }

  TEST(HOGDetectorTests, getMaxBodyPartHeightWidth)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> HFrames = project.getFrames();

    //Counting a keyframes
    //int KeyframesCount = 0;
    int FirstKeyframe = 0;

    //Copy image and skeleton from first keyframe
    Mat image = HFrames[FirstKeyframe]->getImage();
    Mat mask = HFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = HFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    //Set blockSize value
    Size blockSize = Size(8, 8);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    Size wndSize = Size(64, 128);
    const int nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;
    bool bGrayImages = false;

    //Calculate expected parts size 
    map<int, Point2f> Locations = getImageLocations(skeleton);
    map <uint32_t, Size> partsSize;
    for (tree <BodyPart>::iterator bodyPart = partTree.begin(); bodyPart != partTree.end(); ++bodyPart)
    {
      int partID = bodyPart->getPartID();
      uint32_t parentID = bodyPart->getParentJoint();
      uint32_t childID = bodyPart->getChildJoint();
      float lwRatio = bodyPart->getLWRatio();
      Point2f delta = Locations[parentID] - Locations[childID];
      float length = static_cast<float>(norm(delta));
      float width = length / lwRatio;
      uint32_t L = static_cast<uint32_t>(ceil(length / blockSize.width)*blockSize.width);
      uint32_t W = static_cast<uint32_t>(ceil(width / blockSize.height)*blockSize.height);
      partsSize.emplace(pair<uint32_t, Size>(partID, Size(L, W)));
    }

    //Calculate actual parts size
    HogDetector D(nbins, wndSize, padding, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType);
    D.m_frames = HFrames;
    map <uint32_t, Size> partsSize_actual = D.getMaxBodyPartHeightWidth(blockSize, 1.0f);
    EXPECT_EQ(partsSize, partsSize_actual);

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    HFrames.clear();
  }

  /*TEST(HOGDetectorTests, PrepareTestData)
  {
      //Load the input data
      TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
      vector<Frame*> HFrames = project.getFrames();

      //Run "train"
      D.m_partSize = D.getMaxBodyPartHeightWidth(HFrames, D.m_blockSize, 1.0f);
      map<string, float> params;
      D.train(HFrames, params);

      //Clear
      //project.TestProjectLoader::~TestProjectLoader();
      HFrames.clear();
  }*/

  TEST(HOGDetectorTests, generateLabel)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> HFrames = project.getFrames();

    //Counting a keyframes
    //int KeyframesCount = 0;
    int FirstKeyframe = 0;

    //Copy image and skeleton from first keyframe
    Mat image = HFrames[FirstKeyframe]->getImage();
    Mat mask = HFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = HFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    //Select body part for testing
    int partID = 7;
    //Copy body part
    BodyPart bodyPart = *skeleton.getBodyPart(partID);
    //Copy part joints 
    BodyJoint* j0 = skeleton.getBodyJoint(bodyPart.getParentJoint());
    BodyJoint* j1 = skeleton.getBodyJoint(bodyPart.getChildJoint());
    //Copy joints locations
    Point2f p0 = j0->getImageLocation();
    Point2f p1 = j1->getImageLocation();

    //Calculate actual value
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    Size wndSize = Size(64, 128);
    const int nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;
    bool bGrayImages = false;
    HogDetector D(nbins, wndSize, padding, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType);
    map<string, float> params;
    //D.frames = HFrames;
    //D.m_partSize = D.getMaxBodyPartHeightWidth(D.m_blockSize, 1.0f);
    D.train(HFrames, params);
    HogDetector::PartModel partModel = D.getPartModels()[0][partID];
    auto detectorHelper = new HogDetectorHelper();
    map <string, float> detectParams;

    LimbLabel label_actual = D.generateLabel(bodyPart, HFrames[FirstKeyframe], p0, p1, detectorHelper, detectParams);

    delete detectorHelper;

    //Create expected LimbLabel value
    Point2f center = 0.5*(p0 + p1);
    float angle = bodyPart.getRotationSearchRange();
    spelRECT<Point2f> rect = bodyPart.getBodyPartRect(p0, p1);
    vector<Point2f> polygon = rect.asVector();
    Score score;
    vector<Score> scores;
    score.setScore(0);
    scores.push_back(score);

    LimbLabel label_expected(partID, center, angle, polygon, scores);

    //Compare
    EXPECT_EQ(label_expected.getLimbID(), label_actual.getLimbID());
    EXPECT_EQ(label_expected.getCenter(), label_actual.getCenter());
    EXPECT_EQ(label_expected.getPolygon(), label_actual.getPolygon());
    EXPECT_EQ(label_expected.getScores()[0].getScore(), label_actual.getScores()[0].getScore());
    //Skeleton S = frames[FirstKeyframe]->getSkeleton();
    //BodyPart *P = S.getBodyPart(partID);
    //EXPECT_EQ(P->getRotationSearchRange(), label_actual.getAngle());
    float expected_angle = (float)(spelHelper::angle2D(1.0f, 0.0f, p1.x - p0.x, p1.y - p0.y) * 180 / M_PI);
    EXPECT_EQ(expected_angle, label_actual.getAngle());

    //Output debug inf
    cout << endl << "LimbID = " << label_expected.getLimbID() << endl;
    cout << "Angle = " << label_actual.getAngle() << endl;
    cout << "Center = " << label_actual.getCenter() << endl;
    cout << "Polygon = " << label_actual.getPolygon() << endl;

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    HFrames.clear();
  }

  TEST(HOGDetectorTests, train)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> HFrames = project.getFrames();

    //Counting a keyframes
    //int KeyframesCount = 0;
    int FirstKeyframe = 0;

    //Copy image and skeleton from first keyframe
    Mat image = HFrames[FirstKeyframe]->getImage();
    Mat mask = HFrames[FirstKeyframe]->getMask();
    Skeleton skeleton = HFrames[FirstKeyframe]->getSkeleton();
    tree <BodyPart> partTree = skeleton.getPartTree();
    tree <BodyJoint> jointsTree = skeleton.getJointTree();

    // Set descriptor parameters
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    const int nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;
    Size wndSize = Size(64, 128);

    //Calculate actual value
    HogDetector D(nbins, wndSize, padding, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType);
    //D.m_partSize = D.getMaxBodyPartHeightWidth(D.m_blockSize, 1.0f);
    map<string, float> params;
    D.train(HFrames, params);

    //Calculate expected value
    vector<vector<float>> allDescriptors;
    for (unsigned int partID = 0; partID < partTree.size(); partID++)
    {
      vector<float> descriptorsValues;
      BodyPart bodyPart = *skeleton.getBodyPart(partID);//Copy body part	
      //BodyJoint* j0 = skeleton.getBodyJoint(bodyPart.getParentJoint());//Copy part joints 
      //BodyJoint* j1 = skeleton.getBodyJoint(bodyPart.getChildJoint());
      /*HOGDescriptor d(D.m_partSize[partID], blockSize, blockStride, cellSize, nbins, derivAperture, wndSigma, histogramNormType, thresholdL2hys, gammaCorrection, nlevels);*/
      HOGDescriptor d(D.m_partSize[partID], D.m_blockSize, D.m_blockStride, D.m_cellSize, D.m_nbins, D.m_derivAperture, D.m_wndSigma, D.m_histogramNormType, D.m_thresholdL2hys, D.m_gammaCorrection, D.m_nlevels);
      d.compute(D.m_partModels[0][partID].partImage, descriptorsValues);
      allDescriptors.push_back(descriptorsValues);
    }

    //Compare	
    for (unsigned int partID = 0; partID < partTree.size(); partID++)
    {
      EXPECT_EQ(allDescriptors[partID].size(), D.m_partModels[0][partID].descriptors.size());
      for (unsigned int i = 0; i < allDescriptors[partID].size(); i++)
        EXPECT_EQ(allDescriptors[partID][i], D.m_partModels[0][partID].descriptors[i]);
      vector <vector <vector <float>>> G = decodeDescriptor(allDescriptors[partID], D.m_partSize[partID], D.m_blockSize, D.m_blockStride, D.m_cellSize, D.m_nbins);
      for (unsigned int i = 0; i < G.size(); i++)
        for (unsigned int k = 0; k < G[i].size(); k++)
          for (unsigned int n = 0; n < G[i][k].size(); n++)
            EXPECT_EQ(G[i][k][n], D.m_partModels[0][partID].gradientStrengths[i][k][n]);
      allDescriptors[partID].clear();
    }
    allDescriptors.clear();

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    HFrames.clear();
  }

  TEST(HOGDetectorTests, detect)
  {
    //Load the input data
    TestProjectLoader project;
    project.Load("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> HFrames = project.getFrames();
    Skeleton SkeletonPattern = HFrames[0]->getSkeleton();

    // Set descriptor parameters
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    Size wndSize = Size(64, 128);
    const int nbins = 9;
    double wndSigma = -1;
    double thresholdL2hys = 0.2;
    bool gammaCorrection = true;
    int nlevels = 64;
    Size wndStride = Size(8, 8);
    Size padding = Size(32, 32);
    int derivAperture = 1;
    int histogramNormType = HOGDescriptor::L2Hys;

    // Run "train"
    HogDetector D(nbins, wndSize, padding, blockSize, blockStride, cellSize, wndSigma, thresholdL2hys, gammaCorrection, nlevels, derivAperture, histogramNormType);  
    map<string, float> params;
    //D.frames = HFrames;
    //D.m_partSize = D.getMaxBodyPartHeightWidth(blockSize, 1.0f);
    long train_t0 = clock();
    D.train(HFrames, params);  
    long train_t1 = clock();

    // Run "detect"
    map<uint32_t, vector<LimbLabel>> Labels;
    HFrames[1]->setSkeleton(SkeletonPattern);
    long detect_t0 = clock();
    Labels = D.detect(HFrames[1], params, Labels);
    long detect_t1 = clock();

    // Set test parameters
    int TopLabelsCount = 4; // Size of "labels top list"
    float TolerableLinearError = 30; // Linear error in pixels
                                                                                     
    // Compare limb labels with ideal bodyparts from keyframe 
    map<int, vector<float>> LinearErrors = LabelsLinearErrors(Labels, SkeletonPattern);
    map<int, vector<float>> AngleErrors = LabelsAngleErrors(Labels, SkeletonPattern);

    // Write debug information into text file
    string OutputFileName = "DetectTest_HOG.txt";
    ofstream fout(OutputFileName);
    fout << "\n  DETECT RESULTS\n\n";
    PutSigificantErrors(fout, LinearErrors, AngleErrors, TopLabelsCount);
    PutLabels(fout, "", Labels, LinearErrors, AngleErrors, TopLabelsCount);
    PutLabels(fout, "", Labels, LinearErrors, AngleErrors, 0);
    fout.close();

    // Cheking the values and put test result
    vector<int> notFoundedParts = selectNotFoundedParts(SkeletonPattern, LinearErrors, Labels, TolerableLinearError, TopLabelsCount);
    cout << "\n EXECUTION TIME\n" << "\nTrain: " << spelHelper::clock_to_ms(train_t1 - train_t0) << " ms\n";
    cout << "Detect: " << spelHelper::clock_to_ms(detect_t1 - detect_t0) << " ms\n";

    cout << "TEST PARAMETERS\n\n";
    cout << "Image size = " << HFrames[0]->getMask().size() << endl;
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

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    HFrames.clear();
  }
  
  TEST(HOGDetectorTests, compare)
  {
    int DescriptorLength = 3780;
    vector<float> descriptor;
    for (int i = 0; i < DescriptorLength; i++)
      descriptor.push_back(2);

    Size wndSize = Size(64, 128);
    Size blockSize = Size(16, 16);
    Size blockStride = Size(8, 8);
    Size cellSize = Size(8, 8);
    const int nbins = 9;

    HogDetector::PartModel X;
    X.gradientStrengths = decodeDescriptor(descriptor, wndSize, blockSize, blockStride, cellSize, nbins);

    HogDetector detector;

    map  <uint32_t, HogDetector::PartModel > _partModels;
    _partModels.emplace(pair <uint32_t, HogDetector::PartModel>(0, X));
    map <uint32_t, map <uint32_t, HogDetector::PartModel>> framePartModels;
    framePartModels.emplace(pair<uint32_t, map <uint32_t, HogDetector::PartModel>>(0, _partModels));

    detector.m_partModels = framePartModels;

    //map <uint32_t, map>
    BodyPart bodyPart;
    bodyPart.setPartID(0);

    float score = detector.compare(bodyPart, X, nbins);
    cout << "Score = " << score << endl;
    EXPECT_EQ(0, score);

    int N = X.gradientStrengths.size()*X.gradientStrengths[0].size()*X.gradientStrengths[0][0].size();

    X.gradientStrengths[0][0][0] += 1;
    score = detector.compare(bodyPart, X, nbins);
    cout << "Score = " << score << endl;
    float x = float(1.0 / (float)N);
    EXPECT_EQ(x, score);

    framePartModels.emplace(pair<uint32_t, map <uint32_t, HogDetector::PartModel>>(0, _partModels));
    score = detector.compare(bodyPart, X, nbins);
    cout << "Score = " << score << endl;
    x = float(1.0 / (float)N);
    EXPECT_EQ(x, score);
  }

  TEST(HOGDetectorTests, getPartModels)
  {
    //Create "PartModels"
    Size imageSize = Size(4, 3);
    const int F = 3, P = 10, L = 20, I = 5, K = 6, N = 7;
    HogDetector::PartModel X;
    map <uint32_t, map <uint32_t, HogDetector::PartModel>> expected_PartModels;
    for (uint32_t f = 0; f < F; f++)
    {
      map <uint32_t, HogDetector::PartModel> temp_Parts;
      for (uint32_t p = 0; p < P; p++)
      {
        HogDetector::PartModel X_temp;
        X_temp.partModelRect = spelRECT<Point2f>(Point2f(float(f), float(f)), Point2f(float(p), float(p)), Point2f(0.0f, 0.0f), Point2f(0.0f, 0.0f));
        X_temp.partImage = Mat(imageSize, CV_8UC3, Scalar(f, p, 0));
        for (int i = 0; i < I; i++)
        {
          vector <vector <float>> temp_k;
          for (int k = 0; k < K; k++)
          {
            vector <float> temp_n;
            for (int n = 0; n < N; n++)
              temp_n.push_back(float(p + f + i + k));
            temp_k.push_back(temp_n);
            temp_n.clear();
          }
          X_temp.gradientStrengths.push_back(temp_k);
          temp_k.clear();
        }
        temp_Parts.emplace(pair<uint32_t, HogDetector::PartModel>(p, X_temp));
      }
      expected_PartModels.emplace(pair<uint32_t, map <uint32_t, HogDetector::PartModel>>(f, temp_Parts));
      temp_Parts.clear();
    }

    //Create "HogDetector"
    HogDetector detector;
    detector.m_partModels = expected_PartModels;

    for (uint32_t f = 0; f < F; f++)
      for (uint32_t p = 0; p < P; p++)
      {
        detector.m_partModels[f][p].partImage.release();
        detector.m_partModels[f][p].partImage = expected_PartModels[f][p].partImage.clone();
      }

    //Get "PartModels"
    map <uint32_t, map <uint32_t, HogDetector::PartModel>> actual_PartModels;
    actual_PartModels = detector.getPartModels();

    //Compare
    for (uint32_t f = 0; f < F; f++)
      for (uint32_t p = 0; p < P; p++)
        for (int l = 0; l < L; l++)
        {
          EXPECT_EQ(expected_PartModels[f][p].partModelRect, detector.m_partModels[f][p].partModelRect);
          EXPECT_EQ(expected_PartModels[f][p].gradientStrengths, detector.m_partModels[f][p].gradientStrengths);
          Size image_size = expected_PartModels[f][p].partImage.size();
          /*bool ImagesIsEqual = true;
          for (int y = 0; y < image_size.height; y++)
          for (int x = 0; x < image_size.width; x++)
          ImagesIsEqual = (expected_PartModels[f][p].partImage.at<Vec3b>(y, x) == actual_PartModels[f][p].partImage.at<Vec3b>(y, x));
          EXPECT_TRUE(ImagesIsEqual);
          */
          EXPECT_EQ(expected_PartModels[f][p].partImage.at<Vec3b>(0, 0), actual_PartModels[f][p].partImage.at<Vec3b>(0, 0));
        }

    expected_PartModels.clear();
  }

  TEST(HOGDetectorTests, getCellSize)
  {
    HogDetector detector;
    Size size = Size(8, 8);
    detector.m_cellSize = size;

    EXPECT_EQ(size, detector.getCellSize());
  }

  TEST(HOGDetectorTests, getNBins)
  {
    HogDetector detector;
    uint8_t nBins = 9;

    EXPECT_EQ(nBins, detector.getnbins());
  }

  TEST(HOGDetectorTests, constructor)
  {
    int id = 0x48440000;
    HogDetector d;
    EXPECT_EQ(id, d.getID());
  }

  TEST(HOGDetectorTests, calculateHog)
  {
    //Prepare test data
    Size winSize(128, 64);
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

    int rows = 64, cols = 128;
    Mat Image(rows, cols, CV_8UC3, Scalar(0, 0, 0));
    cv::ellipse(Image, Point(0.5f*cols, 0.5f*rows), Size(0.375f*cols, 0.375f*rows), 0.0, 0.0, 360.0, Scalar(255, 255, 255), 1, 0, 0);

    cvtColor(Image, Image, CV_RGB2GRAY);

    vector<float> descriptors;
    
    //Create expected value
    vector<vector<vector<float>>> Expected;
    HOGDescriptor d(winSize, blockSize, blockStride, cellSize, nBins, derivAper, winSigma, histogramNormType, L2HysThresh, gammaCorrection, nLevels);
    d.compute(Image, descriptors);
    Expected = averageGradientStrengths(Image, descriptors, winSize, blockSize, blockStride, cellSize, nBins, derivAper, winSigma, histogramNormType, L2HysThresh, gammaCorrection, nLevels);

    //Create actual value
    HogDetector HOGDetector;
    vector<vector<vector<float>>> Actual;
    Actual = HOGDetector.calculateHog(Image, descriptors, winSize, blockSize, blockStride, cellSize, nBins, derivAper, winSigma, histogramNormType, L2HysThresh, gammaCorrection, nLevels);
    //Actual = decodeDescriptor(descriptors, winSize, blockSize, blockStride, cellSize, nBins);

    //Compare
    //EXPECT_EQ(Expected, Actual);
    ASSERT_EQ(Expected.size(), Actual.size());
    ASSERT_EQ(Expected[0].size(), Actual[0].size());
    ASSERT_EQ(Expected[0][0].size(), Actual[0][0].size());
    bool GradientStrengthsEqual = true;
    for (int y = 0; y < Expected.size(); y++)
      for (int x = 0; x < Expected[y].size(); x++)
        for (int n = 0; n<Expected[y][x].size(); n++)
          if(Expected[y][x][n] != Actual[y][x][n])
          {
            GradientStrengthsEqual = false;
            //cout <<  "strength[" << y << ", " << x << ", " << ", " << n << "]: expected " << Expected[y][x][n] << ", actual " << Actual[y][x][n] << endl;
          }   
    EXPECT_TRUE(GradientStrengthsEqual);
    
    //Put result
    if(!GradientStrengthsEqual)
    {
      cout << endl;
      cout << "winSize = " << winSize << endl;
      cout << "blockSize = " << blockSize << endl;
      cout << "blockStride = " << blockStride << endl;
      cout << "cellSize = " << cellSize << endl;
      cout << "nBins = " << nBins << endl;
      cout << "derivAper = " << derivAper << endl;
      cout << "winSigma = " << winSigma << endl;
      cout << "histogramNormType = " << histogramNormType << endl;
      cout << "L2HysThresh = " << L2HysThresh << endl;
      cout << "gammaCorrection = " << gammaCorrection << endl;
      cout << "gammaCorrection = " << gammaCorrection << endl;
      cout << "nLevels = " << nLevels << endl;
      cout << endl;

      cout << "Expected:" << endl;
      for (int y = 0; y < Expected.size(); y++)
        for (int x = 0; x < Expected[y].size(); x++)
        {
          cout << Point2f(x, y) << ": ";
          for (int n = 0; n < Expected[y][x].size(); n++)
            cout << Expected[y][x][n] << ", ";
          cout << endl;
        }
      cout << endl;
      cout << "Actual:" << endl;
      for (int y = 0; y < Actual.size(); y++)
        for (int x = 0; x < Actual[y].size(); x++)
        {
          cout << Point2f(x, y) << ": ";
          for (int n = 0; n < Actual[y][x].size(); n++)
            cout << Actual[y][x][n] << ", ";
          cout << endl;
        }
     }

    //Clear 
    for (int y = 0; y < Expected.size(); y++)
    {
      for (int x = 0; x < Expected[y].size(); x++)
        for (int n = 0; n < Expected[y][x].size(); n++)
        {
          Expected[y][x].clear();
          Actual[y][x].clear();
        }
      Expected[y].clear();
      Actual[y].clear();
    }
    Expected.clear();
    Actual.clear();
  }
}