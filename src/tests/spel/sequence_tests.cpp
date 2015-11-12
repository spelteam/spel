#include "sequence.hpp"
#include <Eigen/StdVector>
#include <gtest/gtest.h>
#include "TestsFunctions.hpp"

namespace SPEL
{
  //Testing "Sequence::Sequence(const Sequence& seq)"
  TEST(SequenceTests, CopyConstructor)
  {
    //Load the input data
    vector<Frame*> expected_frames = LoadTestProject("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
  
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
          MasksIsEqual = (mask0.at<Vec3b>(y, x) == mask1.at<Vec3b>(y, x));
        }
      EXPECT_TRUE(ImagesIsEqual);
      EXPECT_TRUE(MasksIsEqual);

      image0.release();
      mask0.release();
      image1.release();
      mask1.release();
    } 
  }

  //Testing "Sequence(int idx, std::string seqName, std::vector<Frame*> seq)"
  TEST(SequenceTests, Constructor)
  {
    //Load the input data
    vector<Frame*> expected_frames = LoadTestProject("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");
  
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
          MasksIsEqual = (mask0.at<Vec3b>(y, x) == mask1.at<Vec3b>(y, x));
        }
      EXPECT_TRUE(ImagesIsEqual);
      EXPECT_TRUE(MasksIsEqual);

      image0.release();
      mask0.release();
      image1.release();
      mask1.release();
    } 
  }

  TEST(SequenceTests, getName)
  {
    //Create expected value
    vector<Frame*> frames;
    string expected_Name = "test sequence";
    Sequence sequence(0, expected_Name, frames);

    //Compare
    EXPECT_EQ(expected_Name, sequence.getName());
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
  }

  //Testing "Sequence(int idx, std::string seqName, std::vector<Frame*> seq)"
  TEST(SequenceTests, setAndGetFames)
  {
    //Load the input data
    vector<Frame*> expected_frames = LoadTestProject("speltests_TestData/CHDTrainTestData/", "trijumpSD_50x41.xml");

    //Create expected value
    Sequence sequence;
    sequence.setFrames(expected_frames);

    //Compare
    vector<Frame*> actual_frames =sequence.getFrames();

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
          MasksIsEqual = (mask0.at<Vec3b>(y, x) == mask1.at<Vec3b>(y, x));
        }
      EXPECT_TRUE(ImagesIsEqual);
      EXPECT_TRUE(MasksIsEqual);

      image0.release();
      mask0.release();
      image1.release();
      mask1.release();
    } 
  }

  TEST(SequenceTests, setAndGetID)
  {
    Sequence sequence;
    int ID = 10;
    sequence.setID(ID);
    EXPECT_EQ(ID, sequence.getID());
  }

}
