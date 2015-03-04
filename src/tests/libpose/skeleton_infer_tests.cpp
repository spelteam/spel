#include <gtest/gtest.h>
#include <skeleton.hpp>
#include <projectLoader.hpp>

#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

TEST(skeletonTest, infer3dTest)
{
  string curFolder;
  char cCurrentPath[FILENAME_MAX];

  if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
  {
    FAIL() << "Can't get current directory";
  }
  curFolder = cCurrentPath;
  if (curFolder.back() != '/')
  {
    curFolder += '/';
  }
#ifdef WINDOWS
  curFolder += "Debug/";
#endif
  curFolder += "posetests_TestData/testdata1/";

  ProjectLoader projectLoader(curFolder);

  string xml = curFolder + "trijumpSD_new.xml";

  if (projectLoader.Load(xml) == false)
  {
    FAIL() << "Can't load project xml: " << xml ;
  }

  vector <Frame*> frames = projectLoader.getFrames();

  for (vector <Frame*>::iterator frame = frames.begin(); frame != frames.end(); ++frame)
  {
    if ((*frame)->getFrametype() != KEYFRAME)
      continue;
    Skeleton skeleton = (*frame)->getSkeleton();
    skeleton.infer3D();
    tree <BodyPart> partTree = skeleton.getPartTree();
    for (tree <BodyPart>::iterator bodyPart = partTree.begin(); bodyPart != partTree.end(); ++bodyPart)
    {
      BodyJoint *parentJoint = skeleton.getBodyJoint(bodyPart->getParentJoint());
      BodyJoint *childJoint = skeleton.getBodyJoint(bodyPart->getChildJoint());
      if (parentJoint->getSpaceLocation() == Point3f(0.0, 0.0, 0.0) || childJoint->getSpaceLocation() == Point3f(0.0, 0.0, 0.0))
      {
        FAIL() << "SpaceLocation can't be (0, 0, 0) 1";
      }
      Point3f point1 = parentJoint->getSpaceLocation();
      Point3f point2 = Point3f(childJoint->getSpaceLocation().x, childJoint->getSpaceLocation().y, 0.0);
      Point3f point3 = childJoint->getSpaceLocation();

      float length12 = PoseHelper::distSquared3d(point1, point2);
      float length23 = PoseHelper::distSquared3d(point2, point3);
      float length13 = PoseHelper::distSquared3d(point1, point3);

      EXPECT_NEAR(length13, length12 + length23, 0.999);
    }
  }

  Sequence sequence;
  sequence.setFrames(frames);
  map <string, float> params;

  sequence.estimateUniformScale(params);

  for (vector <Frame*>::iterator frame = frames.begin(); frame != frames.end(); ++frame)
  {
    if ((*frame)->getFrametype() != KEYFRAME)
      continue;
    Skeleton skeleton = (*frame)->getSkeleton();
    skeleton.infer3D();
    tree <BodyPart> partTree = skeleton.getPartTree();
    for (tree <BodyPart>::iterator bodyPart = partTree.begin(); bodyPart != partTree.end(); ++bodyPart)
    {
      BodyJoint *parentJoint = skeleton.getBodyJoint(bodyPart->getParentJoint());
      BodyJoint *childJoint = skeleton.getBodyJoint(bodyPart->getChildJoint());
      if (parentJoint->getSpaceLocation() == Point3f(0.0, 0.0, 0.0) || childJoint->getSpaceLocation() == Point3f(0.0, 0.0, 0.0))
      {
        FAIL() << "SpaceLocation can't be (0, 0, 0) 2";
      }
      Point3f point1 = parentJoint->getSpaceLocation();
      Point3f point2 = Point3f(childJoint->getSpaceLocation().x, childJoint->getSpaceLocation().y, 0.0);
      Point3f point3 = childJoint->getSpaceLocation();

      float length12 = PoseHelper::distSquared3d(point1, point2);
      float length23 = PoseHelper::distSquared3d(point2, point3);
      float length13 = PoseHelper::distSquared3d(point1, point3);

      EXPECT_NEAR(length13, length12 + length23, 0.999);
    }
  }

  sequence.computeInterpolation(params);

  for (vector <Frame*>::iterator frame = frames.begin(); frame != frames.end(); ++frame)
  {
    //TODO(Vitalii Koshura): Remove this after correct work of interpolation
    /*if ((*frame)->getFrametype() != KEYFRAME)
      continue;*/
    //check if frame has a keyframe before AND after
    bool hasPrevAnchor=false, hasFutureAnchor=false;
    //before
    for(vector<Frame*>::reverse_iterator prevAnchor(frame); prevAnchor!=frames.rend(); ++prevAnchor)
    {
        if((*prevAnchor)->getFrametype()==KEYFRAME || (*prevAnchor)->getFrametype()==LOCKFRAME)
        {
            hasPrevAnchor=true;
            break;
        }
    }
    //after
    for(vector<Frame*>::iterator futureAnchor=frame; futureAnchor!=frames.end(); ++futureAnchor)
    {
        if((*futureAnchor)->getFrametype()==KEYFRAME || (*futureAnchor)->getFrametype()==LOCKFRAME)
        {
            hasFutureAnchor=true;
            break;
        }
    }
    if(!hasPrevAnchor || !hasFutureAnchor)
        continue;
    Skeleton skeleton = (*frame)->getSkeleton();
    skeleton.infer3D();
    tree <BodyPart> partTree = skeleton.getPartTree();
    for (tree <BodyPart>::iterator bodyPart = partTree.begin(); bodyPart != partTree.end(); ++bodyPart)
    {
      BodyJoint *parentJoint = skeleton.getBodyJoint(bodyPart->getParentJoint());
      BodyJoint *childJoint = skeleton.getBodyJoint(bodyPart->getChildJoint());
      if (parentJoint->getSpaceLocation() == Point3f(0.0, 0.0, 0.0) || childJoint->getSpaceLocation() == Point3f(0.0, 0.0, 0.0))
      {
        FAIL() << "SpaceLocation can't be (0, 0, 0) 3";
      }
      Point3f point1 = parentJoint->getSpaceLocation();
      Point3f point2 = Point3f(childJoint->getSpaceLocation().x, childJoint->getSpaceLocation().y, 0.0);
      Point3f point3 = childJoint->getSpaceLocation();

      float length12 = PoseHelper::distSquared3d(point1, point2);
      float length23 = PoseHelper::distSquared3d(point2, point3);
      float length13 = PoseHelper::distSquared3d(point1, point3);

      EXPECT_NEAR(length13, length12 + length23, 0.999);
    }
  }

  sequence.estimateUniformScale(params);

  for (vector <Frame*>::iterator frame = frames.begin(); frame != frames.end(); ++frame)
  {
    //TODO(Vitalii Koshura): Remove this after correct work of interpolation
    /*if ((*frame)->getFrametype() != KEYFRAME)
      continue;*/
      bool hasPrevAnchor=false, hasFutureAnchor=false;
      //before
      for(vector<Frame*>::reverse_iterator prevAnchor(frame); prevAnchor!=frames.rend(); ++prevAnchor)
      {
          if((*prevAnchor)->getFrametype()==KEYFRAME || (*prevAnchor)->getFrametype()==LOCKFRAME)
          {
              hasPrevAnchor=true;
              break;
          }
      }
      //after
      for(vector<Frame*>::iterator futureAnchor=frame; futureAnchor!=frames.end(); ++futureAnchor)
      {
          if((*futureAnchor)->getFrametype()==KEYFRAME || (*futureAnchor)->getFrametype()==LOCKFRAME)
          {
              hasFutureAnchor=true;
              break;
          }
      }
      if(!hasPrevAnchor || !hasFutureAnchor)
          continue;
    Skeleton skeleton = (*frame)->getSkeleton();
    skeleton.infer3D();
    tree <BodyPart> partTree = skeleton.getPartTree();
    for (tree <BodyPart>::iterator bodyPart = partTree.begin(); bodyPart != partTree.end(); ++bodyPart)
    {
      BodyJoint *parentJoint = skeleton.getBodyJoint(bodyPart->getParentJoint());
      BodyJoint *childJoint = skeleton.getBodyJoint(bodyPart->getChildJoint());
      if (parentJoint->getSpaceLocation() == Point3f(0.0, 0.0, 0.0) || childJoint->getSpaceLocation() == Point3f(0.0, 0.0, 0.0))
      {
        FAIL() << "SpaceLocation can't be (0, 0, 0) 4";
      }
      Point3f point1 = parentJoint->getSpaceLocation();
      Point3f point2 = Point3f(childJoint->getSpaceLocation().x, childJoint->getSpaceLocation().y, 0.0);
      Point3f point3 = childJoint->getSpaceLocation();

      float length12 = PoseHelper::distSquared3d(point1, point2);
      float length23 = PoseHelper::distSquared3d(point2, point3);
      float length13 = PoseHelper::distSquared3d(point1, point3);

      EXPECT_NEAR(length13, length12 + length23, 0.999);
    }
  }

}
