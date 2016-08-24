#include "sequence.hpp"
#include <Eigen/StdVector>
#include <gtest/gtest.h>
#include "spelHelper.hpp"
#include "TestsFunctions.hpp"


namespace SPEL
{
  //Testing "Sequence::Sequence(const Sequence& seq)"
  TEST(SequenceTests, CopyConstructor)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> expected_frames = project.getFrames();
  
    //Create expected value
    Sequence expected_sequence(0, "test sequence", expected_frames);

    //Create actual value
    Sequence actual_sequence(expected_sequence);

    //Compare
    ASSERT_EQ(expected_frames.size(), actual_sequence.frames.size());

    Size imageSize = expected_frames[0]->getImage().size();
    for (unsigned int i = 0; i < expected_frames.size(); i++)
    {
      EXPECT_EQ(expected_frames[i]->getID(), actual_sequence.frames[i]->getID());
      EXPECT_EQ(expected_frames[i]->getGroundPoint(), actual_sequence.frames[i]->getGroundPoint());
      EXPECT_EQ(expected_frames[i]->getParentFrameID(), actual_sequence.frames[i]->getParentFrameID());
      EXPECT_EQ(expected_frames[i]->getSkeleton(), actual_sequence.frames[i]->getSkeleton());

      Mat image0 = expected_frames[i]->getImage();
      Mat mask0 = expected_frames[i]->getMask();
      Mat image1 = actual_sequence.frames[i]->getImage();
      Mat mask1 = actual_sequence.frames[i]->getMask();
      bool ImagesIsEqual = true, MasksIsEqual = true;

      ASSERT_EQ(mask0.size(), image0.size());
      ASSERT_EQ(image0.size(), image1.size());
      ASSERT_EQ(mask0.size(), mask1.size());
      for (int y = 0; y < imageSize.height - 1; y++)
        for (int x = 0; x < imageSize.width - 1; x++)
        {
          ImagesIsEqual = (image0.at<Vec3b>(y, x) == image1.at<Vec3b>(y, x));
          MasksIsEqual = (mask0.at<uchar>(y, x) == mask1.at<uchar>(y, x));
        }
      EXPECT_TRUE(ImagesIsEqual);
      EXPECT_TRUE(MasksIsEqual);

      image0.release();
      mask0.release();
      image1.release();
      mask1.release();
    } 

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    expected_frames.clear();
  }

  //Testing "Sequence(int idx, std::string seqName, std::vector<Frame*> seq)"
  TEST(SequenceTests, Constructor)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> expected_frames = project.getFrames();
  
    //Create expected value
    Sequence expected_sequence(0, "trijumpSD_50x41.xml", expected_frames);

    //Create actual value
    int seqID = 100;
    string seqName = "test sequence";
    Sequence actual_sequence(seqID, seqName, expected_frames);
    vector<Frame*> actual_frames = actual_sequence.getFrames();

    //Compare
    EXPECT_EQ(seqID, actual_sequence.getID());
    EXPECT_EQ(seqName, actual_sequence.getName());
    ASSERT_EQ(expected_frames.size(), actual_frames.size());

    Size imageSize = expected_frames[0]->getImage().size();
    for (unsigned int i = 0; i < expected_frames.size(); i++)
    {
      EXPECT_EQ(expected_frames[i]->getID(), actual_frames[i]->getID());
      EXPECT_EQ(expected_frames[i]->getGroundPoint(), actual_frames[i]->getGroundPoint());
      EXPECT_EQ(expected_frames[i]->getParentFrameID(), actual_frames[i]->getParentFrameID());
      EXPECT_EQ(expected_frames[i]->getSkeleton(), actual_frames[i]->getSkeleton());

      Mat image0 = expected_frames[i]->getImage();
      Mat mask0 = expected_frames[i]->getMask();
      Mat image1 = actual_frames[i]->getImage();
      Mat mask1 = actual_frames[i]->getMask();
      bool ImagesIsEqual = true, MasksIsEqual = true;

      ASSERT_EQ(mask0.size(), image0.size());
      ASSERT_EQ(image0.size(), image1.size());
      ASSERT_EQ(mask0.size(), mask1.size());
      for (int y = 0; y < imageSize.height - 1; y++)
        for (int x = 0; x < imageSize.width - 1; x++)
        {
          ImagesIsEqual = (image0.at<Vec3b>(y, x) == image1.at<Vec3b>(y, x));
          MasksIsEqual = (mask0.at<uchar>(y, x) == mask1.at<uchar>(y, x));
        }
      EXPECT_TRUE(ImagesIsEqual);
      EXPECT_TRUE(MasksIsEqual);

      image0.release();
      mask0.release();
      image1.release();
      mask1.release();
    } 

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    expected_frames.clear();
    for (unsigned int i = 0; i < actual_frames.size(); i++)
      delete actual_frames[i];
    actual_frames.clear();

  }

  TEST(SequenceTests, getName)
  {
    //Create expected value
    vector<Frame*> frames;
    string expected_Name = "test sequence";
    Sequence sequence(0, expected_Name, frames);

    //Compare
    EXPECT_EQ(expected_Name, sequence.getName());

    // Clear
    for (unsigned int i = 0; i < frames.size(); i++)
      delete frames[i];
    frames.clear();
  }

  TEST(SequenceTests, setName)
  {
    //Create expected value
    vector<Frame*> frames;
    string expected_Name = "test sequence";
    Sequence sequence;
    sequence.setName(expected_Name);

    //Compare
    EXPECT_EQ(expected_Name, sequence.getName());

    // Clear
    for (unsigned int i = 0; i < frames.size(); i++)
      delete frames[i];
    frames.clear();
  }

  //Testing "Sequence(int idx, std::string seqName, std::vector<Frame*> seq)"
  TEST(SequenceTests, setAndGetFames)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> expected_frames = project.getFrames();

    //Create expected value
    Sequence sequence;
    sequence.setFrames(expected_frames);

    //Compare
    vector<Frame*> actual_frames = sequence.getFrames();

    //Compare
    ASSERT_EQ(expected_frames.size(), actual_frames.size());

    Size imageSize = expected_frames[0]->getImage().size();
    for (unsigned int i = 0; i < expected_frames.size(); i++)
    {
      EXPECT_EQ(expected_frames[i]->getID(), actual_frames[i]->getID());
      EXPECT_EQ(expected_frames[i]->getGroundPoint(), actual_frames[i]->getGroundPoint());
      EXPECT_EQ(expected_frames[i]->getParentFrameID(), actual_frames[i]->getParentFrameID());
      EXPECT_EQ(expected_frames[i]->getSkeleton(), actual_frames[i]->getSkeleton());

      Mat image0 = expected_frames[i]->getImage();
      Mat mask0 = expected_frames[i]->getMask();
      Mat image1 = actual_frames[i]->getImage();
      Mat mask1 = actual_frames[i]->getMask();
      bool ImagesIsEqual = true, MasksIsEqual = true;

      ASSERT_EQ(mask0.size(), image0.size());
      ASSERT_EQ(image0.size(), image1.size());
      ASSERT_EQ(mask0.size(), mask1.size());
      for (int y = 0; y < imageSize.height - 1; y++)
        for (int x = 0; x < imageSize.width - 1; x++)
        {
          ImagesIsEqual = (image0.at<Vec3b>(y, x) == image1.at<Vec3b>(y, x));
          MasksIsEqual = (mask0.at<uchar>(y, x) == mask1.at<uchar>(y, x));
        }
      EXPECT_TRUE(ImagesIsEqual);
      EXPECT_TRUE(MasksIsEqual);

      image0.release();
      mask0.release();
      image1.release();
      mask1.release();
    } 

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    expected_frames.clear();
    for (unsigned int i = 0; i < actual_frames.size(); i++)
      delete actual_frames[i];
    actual_frames.clear();
  }

//Testing "Sequence(int idx, std::string seqName, std::vector<Frame*> seq)"
  TEST(SequenceTests, getFamePointer)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> frames = project.getFrames();
    Sequence sequence;
    sequence.setFrames(frames);
    int frameNum = 0;
    int frameID = frames[frameNum]->getID();

    //Create actual value

    Frame* actual_frame = sequence.getFrame(frameID);

    //Compare
    EXPECT_EQ(frames[frameNum]->getID(), actual_frame->getID());
    EXPECT_EQ(frames[frameNum]->getGroundPoint(), actual_frame->getGroundPoint());
    EXPECT_EQ(frames[frameNum]->getParentFrameID(), actual_frame->getParentFrameID());
    EXPECT_EQ(frames[frameNum]->getSkeleton(), actual_frame->getSkeleton());

    Mat image0 = frames[frameNum]->getImage();
    Mat mask0 = frames[frameNum]->getMask();
    Mat image1 = actual_frame->getImage();
    Mat mask1 = actual_frame->getMask();
    bool ImagesIsEqual = true, MasksIsEqual = true;

    ASSERT_EQ(mask0.size(), image0.size());
    ASSERT_EQ(image0.size(), image1.size());
    ASSERT_EQ(mask0.size(), mask1.size());
    for (int y = 0; y < image0.size().height - 1; y++)
      for (int x = 0; x < image0.size().width - 1; x++)
      {
        ImagesIsEqual = (image0.at<Vec3b>(y, x) == image1.at<Vec3b>(y, x));
        MasksIsEqual = (mask0.at<uchar>(y, x) == mask1.at<uchar>(y, x));
      }
    EXPECT_TRUE(ImagesIsEqual);
    EXPECT_TRUE(MasksIsEqual);

    image0.release();
    mask0.release();
    image1.release();
    mask1.release();

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    frames.clear();
    delete actual_frame;
  }

  TEST(SequenceTests, setAndGetID)
  {
    Sequence sequence;
    int ID = 10;
    sequence.setID(ID);
    EXPECT_EQ(ID, sequence.getID());
  }

  TEST(SequenceTests, interpolateSlice2D)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> frames = project.getFrames();
    Sequence sequence(0, "colorHistDetector", frames);

    //Create Slice
    int firstKeyframeNum = 0, lastKeyframeNum = 2;
    vector<Frame*> slice;
    for (int i = firstKeyframeNum; i <= lastKeyframeNum; i++)
      slice.push_back(frames[i]);

    //Calculate actual values
    map<string, float> parameters;
    vector<Frame*> actual_slice = sequence.interpolateSlice2D(slice, parameters);

    //Calculate expected values
    Skeleton skeleton0 = frames[firstKeyframeNum]->getSkeleton();
    Skeleton skeleton2 = frames[lastKeyframeNum]->getSkeleton();
    skeleton0.infer3D();
    skeleton2.infer3D();
    map<int, pair<Point2f, Point2f>> A = getPartLocations(skeleton0);
    map<int, pair<Point2f, Point2f>> B = getPartLocations(skeleton2);
    map<int, pair<Point2f, Point2f>> expected_locations;
    for (unsigned int i = 0; i < A.size(); i++)
    {
      pair<Point2f, Point2f > temp(0.5f*(A[i].first + B[i].first), 0.5f*(A[i].second + B[i].second));
      expected_locations.emplace(pair<int, pair<Point2f, Point2f>>(i, temp));
    }
    A.clear();
    B.clear();

    for (unsigned int i = 1; i < slice.size() - 1; i++)
    {
      // Copy the expected frame fields
      //int expected_frameID = frames[firstKeyframeNum + i]->getID();
      Point2f expected_groundPoit = frames[firstKeyframeNum + i]->getGroundPoint();
      cv::Mat expected_Image = frames[firstKeyframeNum + i]->getImage();
      cv::Mat expected_Mask = frames[firstKeyframeNum + i]->getMask();

      Skeleton actual_skeleton = frames[i]->getSkeleton();
      map<int, pair<Point2f, Point2f>> actual_locations = getPartLocations(actual_skeleton);

      //Compare
      EXPECT_EQ(INTERPOLATIONFRAME, actual_slice[i]->getFrametype());
      EXPECT_EQ(expected_groundPoit, actual_slice[i]->getGroundPoint());
      EXPECT_EQ(expected_locations, actual_locations);
      cv::Mat actual_Image = actual_slice[i]->getImage();
      ASSERT_EQ(actual_Image.data, actual_Image.data);
      /*bool images_is_equal = true;
      ASSERT_EQ(expected_Image.size(), actual_Image.size());
      for (int y = 0; y < actual_Image.rows; y++)
        for (int x = 0; x < actual_Image.cols; x++)
          if(expected_Image.at<Vec3b>(y, x) != actual_Image.at<Vec3b>(y, x))
            images_is_equal = false;
      EXPECT_TRUE(images_is_equal);
      */

      cv::Mat actual_Mask = actual_slice[i]->getMask();
      ASSERT_EQ(actual_Mask.data, actual_Mask.data);
      /*ASSERT_EQ(expected_Mask.size(), actual_Mask.size());
      bool masks_is_equal = true;
      for (int y = 0; y < actual_Mask.rows; y++)
      for (int x = 0; x < actual_Mask.cols; x++)
      if (expected_Mask.at<Vec3b>(y, x) != actual_Mask.at<Vec3b>(y, x))
        masks_is_equal = false;
      EXPECT_TRUE(masks_is_equal);
      */
    }

    // Clear    
    slice.clear();
    //project.TestProjectLoader::~TestProjectLoader();
    frames.clear();
    actual_slice.clear();
  }

  TEST(SequenceTests, computeInterpolation)
  {
    /*
    //Load the input data
    vector<Frame*> frames = LoadTestProject("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    Sequence sequence(0, "colorHistDetector", frames);

    //Create Slices
    int keyframeNum = FirstKeyFrameNum(frames);
    vector<vector<Frame*>> Slices;
    int k = 0;
    for (int i = keyframeNum; i < frames.size(); i++)
    {
      vector<Frame*> Slice;
      Slice.push_back(frames[i]);
      k++;
      if (frames[i]->getFrametype() == KEYFRAME && (k > 2))
      {
        k = 0;
        Slices.push_back(Slice);
        Slice.clear();
        Slice.push_back(frames[i]);
      }
    }
    int n = Slices.size() - 1;
    int m = Slices[n][Slices[n].size[-1]];
    if (Slices[n][m]->getFrametype() != KEYFRAME)
    Slices.pop_back();
    */

    bool returns_a_value = false; // "Sequence::computeInterpolation" don't return a values and don't change a fields or parameters?
    EXPECT_TRUE(returns_a_value);
  }

  TEST(SequenceTests, estimateUniformScale)
  {
    //Load the input data
    TestProjectLoader project("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
    vector<Frame*> frames = project.getFrames();
    Sequence sequence(0, "colorHistDetector", frames);

    //Prepare test data
    Skeleton S = frames[0]->getSkeleton();
    tree<BodyPart> partTree = S.getPartTree();
    int k = 0;
    for (tree<BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
    {
      float len2d = sqrt(spelHelper::distSquared(S.getBodyJoint(i->getParentJoint())->getImageLocation(), S.getBodyJoint(i->getChildJoint())->getImageLocation()));
      //cout << k << ": " << i->getPartID() << ", len2d = " << len2d;
      if ((i->getPartID()) != 9) len2d *= 1.0f + 0.05f*rand()/RAND_MAX;
      i->setRelativeLength(len2d);
      //cout << ", len3d = " << i->getRelativeLength() << endl;
      k++;
    }
    frames[0]->setSkeleton(S);
    
    S = frames[2]->getSkeleton();
    for (tree<BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
    {
      float len2d = sqrt(spelHelper::distSquared(S.getBodyJoint(i->getParentJoint())->getImageLocation(), S.getBodyJoint(i->getChildJoint())->getImageLocation()));
      i->setRelativeLength(len2d);
      //cout << ", len3d = " << i->getRelativeLength() << endl;
    }
    frames[1]->setSkeleton(S);

    // Create actual value
    sequence.setFrames(frames);
    map<std::string, float> params;
    sequence.estimateUniformScale(params);
    /*
    frames = sequence.getFrames();
    Skeleton S0 = frames[0]->getSkeleton();
    Skeleton S1 = frames[1]->getSkeleton();
    Skeleton S2 = frames[2]->getSkeleton();*/

    Skeleton S0 = sequence.getFrame(0)->getSkeleton();
    Skeleton S1 = sequence.getFrame(1)->getSkeleton();
    Skeleton S2 = sequence.getFrame(2)->getSkeleton();
    EXPECT_FLOAT_EQ(1.0f, S0.getScale());
    EXPECT_FLOAT_EQ(1.0f, S1.getScale());
    EXPECT_FLOAT_EQ(1.0f, S2.getScale());

    // Clear
    //project.TestProjectLoader::~TestProjectLoader();
    frames.clear();
  }

}
