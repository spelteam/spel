#include <iostream>
#include <fstream>

#include <tinyxml2.h>
#include <opencv2/opencv.hpp>

#include <colorHistDetector.hpp>
#include <keyframe.hpp>
#include <interpolation.hpp>

using namespace std;
using namespace tinyxml2;

const string bodyJointsNode = "BodyJoints";
const string bodyPartsNode = "BodyParts";
const string framesNode = "Frames";
const string bodyJointNode = "BodyJoint";
const string bodyPartNode = "BodyPart";
const string frameNode = "Frame";
const string bodyJointIdParam = "id";
const string bodyJointNameParam = "name";
const string bodyPartNameParam = "name";
const string bodyPartIdParam = "id";
const string bodyPartParentJointIdParam = "parentJointId";
const string bodyPartChildJointIdParam = "childJointId";
const string bodyPartDepthSignParam = "depthSign";
const string bodyPartExpectedDistanceParam = "expectedDistance";
const string frameIdParam = "id";
const string frameImgPathParam = "imgPath";
const string frameMaskPathParam = "maskPath";
const string frameCamPathParam = "camPath";
const string frameIsKeyframeParam = "isKeyframe";
const string bodyJointXParam = "x";
const string bodyJointYParam = "y";

int main (int argc, char **argv)
{
  if (argc != 3) 
  {
    cout << "Usage colorHistDetectorTest [project.xml] [out directory]" << endl;
    return -1;
  }
  XMLDocument project;
  if (!project.LoadFile(argv[1]))
  {
    cerr << "Could not load the project from " << argv[1] << endl;
  }
  XMLElement *root = project.RootElement();
  XMLNode *bodyJoints = root->FirstChildElement(bodyJointsNode.c_str());
  if (bodyJoints != 0)
  {
    bodyJoints = bodyJoints->FirstChildElement(bodyJointNode.c_str());
    if (bodyJoints == 0)
    {
      cerr << "Incorrect xml structure" << endl;
      return -1;
    }
  }
  else
  {
    cerr << "Incorrect xml structure" << endl;
    return -1;
  }
  XMLNode *bodyParts = root->FirstChildElement(bodyPartsNode.c_str());
  if (bodyParts != 0)
  {
    bodyParts = bodyParts->FirstChildElement(bodyPartNode.c_str());
    if (bodyParts == 0)
    {
      cerr << "Incorrect xml structure" << endl;
      return -1;
    }
  }
  else
  {
    cerr << "Incorrect xml structure" << endl;
    return -1;
  }
  XMLNode *frames = root->FirstChildElement(framesNode.c_str());
  if (frames != 0)
  {
    frames = frames->FirstChildElement(frameNode.c_str());
    if (frames == 0)
    {
      cerr << "Incorrect xml structure" << endl;
      return -1;
    }
  }
  else
  {
    cerr << "Incorrect xml structure" << endl;
    return -1;
  }
  tree <BodyJoint> trBodyJoints, trBodyJointsCopy;
  tree <BodyJoint>::iterator topBodyJoints;
  topBodyJoints = trBodyJoints.begin();
  while(true)
  {
    if (bodyJoints == 0) break;
    BodyJoint joint;
    XMLElement *e = bodyJoints->ToElement();
    int id;
    id = e->IntAttribute(bodyJointIdParam.c_str());
    joint.setLimbID(id);
    string name;
    name = e->Attribute(bodyJointNameParam.c_str());
    joint.setJointName(name);
    trBodyJoints.insert(topBodyJoints, joint);
    bodyJoints = bodyJoints->NextSiblingElement();
  }
  tree <BodyPart> trBodyParts;
  tree <BodyPart>::iterator topBodyParts;
  topBodyParts = trBodyParts.begin();
  while (true)
  {
    if (bodyParts == 0) break;
    BodyPart part;
    XMLElement *e = bodyParts->ToElement();
    string parentJointId, childJointId, name;
    int id;
    float expectedDistance;
    bool depthSign;
    id = e->IntAttribute(bodyPartIdParam.c_str());
    name = e->Attribute(bodyPartNameParam.c_str());
    parentJointId = e->Attribute(bodyPartParentJointIdParam.c_str());
    childJointId = e->Attribute(bodyPartChildJointIdParam.c_str());
    depthSign = e->BoolAttribute(bodyPartDepthSignParam.c_str());
    expectedDistance = e->FloatAttribute(bodyPartExpectedDistanceParam.c_str());
    BodyJoint *parentJoint = 0, *childJoint = 0;
    for (topBodyJoints = trBodyJoints.begin(); topBodyJoints != trBodyJoints.end(); ++topBodyJoints)
    {
      if (parentJoint == 0 && topBodyJoints->getJointName() == parentJointId)
      {
        parentJoint = &*topBodyJoints;
      }
      else if (childJoint == 0 && topBodyJoints->getJointName() == childJointId)
      {
        childJoint = &*topBodyJoints;
      }
      if (parentJoint != 0 && childJoint != 0) break;
    }
    if (parentJoint == 0 || childJoint == 0)
    {
      cerr << "Could not find BodyJoint " << ((parentJoint == 0) ? parentJointId : childJointId) << endl;
      return -1;
    }
    part.setPartID(id);
    part.setPartName(name);
    part.setParentJoint(parentJoint);
    part.setChildJoint(childJoint);
    part.setIsOccluded(depthSign);
    part.setSpaceLength(expectedDistance);
    trBodyParts.insert(topBodyParts, part);
    bodyParts = bodyParts->NextSiblingElement();
  }
  vector <Frame*> vFrames;
  while (true)
  {
    if (frames == 0) break;
    Frame *f = 0;
    XMLElement *e = frames->ToElement();
    string imgPath, maskPath, camPath;
    int id;
    bool isKeyFrame;
    id = e->IntAttribute(frameIdParam.c_str());
    imgPath = e->Attribute(frameImgPathParam.c_str());
    maskPath = e->Attribute(frameMaskPathParam.c_str());
    camPath = e->Attribute(frameCamPathParam.c_str());
    isKeyFrame = e->BoolAttribute(frameIsKeyframeParam.c_str());
    if (isKeyFrame == true)
    {
      f = new Keyframe();
    }
    else
    {
      f = new Interpolation();
    }
    f->setID(id);
    Mat image = imread(imgPath, CV_LOAD_IMAGE_COLOR);
    if (!image.data)
    {
      cerr << "Could not find file " << imgPath << endl;
      return -1;
    }
    f->setImage(image);
    Mat mask = imread(maskPath, CV_LOAD_IMAGE_COLOR);
    if (!mask.data)
    {
      cerr << "Could not find file " << maskPath << endl;
      return -1;
    }
    f->setMask(mask);
    bodyJoints = frames->FirstChildElement(bodyJointNode.c_str());
    copy(trBodyJoints.begin(), trBodyJoints.end(), trBodyJointsCopy.begin());
    topBodyJoints = trBodyJointsCopy.begin();
    while (true)
    {
      if (bodyJoints == 0) break;
      XMLElement *e = bodyJoints->ToElement();
      string id;
      float x, y;
      id = e->Attribute(bodyJointIdParam.c_str());
      x = e->FloatAttribute(bodyJointXParam.c_str());
      y = e->FloatAttribute(bodyJointYParam.c_str());
      BodyJoint *joint;
      for (topBodyJoints = trBodyJointsCopy.begin(); topBodyJoints != trBodyJointsCopy.end(); ++topBodyJoints)
      {
        if (joint == 0 && topBodyJoints->getJointName() == id)
        {
          joint = &*topBodyJoints;
        }
        if (joint != 0) break;
      }
      if (joint == 0)
      {
        cerr << "Could not find BodyJoint " << id;
        return -1;
      }
      Point2f imgLocation = Point2f(x, y);
      joint->setImageLocation(imgLocation);
      bodyJoints = bodyJoints->NextSiblingElement();
    }
    Skeleton skeleton;
    skeleton.setPartTree(trBodyParts);
    skeleton.setJointTree(trBodyJoints);
    f->setSkeleton(skeleton);
    vFrames.push_back(f);
    frames = frames->NextSiblingElement();
  }
  ColorHistDetector detector;
  map <string, float> params;
  detector.train(vFrames, params);

  vector <vector <LimbLabel> >::iterator lls;
  vector <LimbLabel>::iterator ls;

  vector <Frame*>::iterator i;
  map <string, float> detectParams;
  for (i = vFrames.begin(); i != vFrames.end(); ++i)
  {
    Frame *f = *i;
    if (f->getFrametype() == INTERPOLATIONFRAME)
    {
      vector <vector <LimbLabel> > labels = detector.detect(f, detectParams);
      uint32_t count = 0;
      for (lls = labels.begin(); lls != labels.end(); ++lls)
      {
        ofstream outFile;
        string outFileName = argv[2];
        if (outFileName[outFileName.size()] != '/')
          outFileName += "/";
        stringstream ss;
        ss << count;
        outFileName += ss.str();
        outFile.open(outFileName);
        for (ls = lls->begin(); ls != lls->end(); ++ls)
        {
          outFile << ls->getLimbID() << " ";
          outFile << ls->getCenter().x << " ";
          outFile << ls->getCenter().y << " ";
          outFile << ls->getAngle() << " ";
          outFile << PoseHelper::distSquared(ls->getPolygon()[0], ls->getPolygon()[2]) << " ";
          outFile << ls->getSumScore() << " ";
          outFile << "0" << " ";
          outFile << "0" << " ";
          outFile << ((ls->getIsOccluded() == true) ? 0 : 1) << std::endl;
        }
        outFile.close();
        count++;
      }
    }
  }
  for (i = vFrames.begin(); i != vFrames.end(); ++i)
  {
    Frame *f = *i;
    if (f != 0)
      delete f;
  }
  return 0;
}

