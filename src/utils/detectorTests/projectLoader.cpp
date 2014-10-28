#include "projectLoader.hpp"
ProjectLoader::ProjectLoader(string _curFolder)
{
  curFolder = _curFolder;
}

void ProjectLoader::SetCurFolder(string _curFolder)
{
  curFolder = _curFolder;
}

vector <Frame*> ProjectLoader::getFrames(void)
{
  return vFrames;
}

bool ProjectLoader::Load(string fileName)
{
  const string projectNode = "Project";
  const string projectName = "name";
  const string projectImgFolderPath = "imgFolderPath";
  const string projectMaskFolderPath = "maskFolderPath";
  const string projectCamFolderPath = "camFolderPath";
  const string projectAllowScaling = "allowScaling";
  const string projectSimMathPath = "simMatPath";
  const string projectExportPath = "exportPath";
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
  const string bodyJointDepthSignParam = "depthSign";
  const string bodyPartExpectedDistanceParam = "expectedDistance";
  const string bodyPartIsOccludedParam = "isOccluded";
  const string frameIdParam = "id";
  const string frameImgPathParam = "imgPath";
  const string frameMaskPathParam = "maskPath";
  const string frameCamPathParam = "camPath";
  const string frameIsKeyframeParam = "isKeyframe";
  const string frameGPX = "gpX";
  const string frameGPY = "gpY";
  const string bodyJointXParam = "x";
  const string bodyJointYParam = "y";

  XMLDocument project;
  XMLError status = project.LoadFile(fileName.c_str());
  if (status != 0)
  {
    cerr << "Could not load the project from " << fileName << endl;
    cerr << "Error code: " << status << endl;
    return false;
  }
  XMLElement *root = project.RootElement();
  if (root == 0 || root->Name() != projectNode)
  {
    cerr << "Incorrect xml structure" << endl;
    if (root == 0)
    {
      cerr << "Empty xml" << endl;
    }
    else
    {
       cerr << "Expect: " << projectNode << endl;
       cerr << "Got: " << root->Name() << endl;
    }
      return false;
  }
  XMLNode *bodyJoints = root->FirstChildElement(bodyJointsNode.c_str());
  if (bodyJoints != 0)
  {
    bodyJoints = bodyJoints->FirstChildElement(bodyJointNode.c_str());
    if (bodyJoints == 0)
    {
      cerr << "Incorrect xml structure" << endl;
      cerr << "Couldn't find BodyJoint structure" << endl;
      return false;
    }
  }
  else
  {
    cerr << "Incorrect xml structure" << endl;
    cerr << "Couldn't find BodyJoints structure" << endl;
    return false;
  }
  XMLNode *bodyParts = root->FirstChildElement(bodyPartsNode.c_str());
  if (bodyParts != 0)
  {
    bodyParts = bodyParts->FirstChildElement(bodyPartNode.c_str());
    if (bodyParts == 0)
    {
      cerr << "Incorrect xml structure" << endl;
      cerr << "Couldn't find BodyPart structure" << endl;
      return false;
    }
  }
  else
  {
    cerr << "Incorrect xml structure" << endl;
    cerr << "Couldn't find BodyParts strunture" << endl;
    return false;
  }
  XMLNode *frames = root->FirstChildElement(framesNode.c_str());
  if (frames != 0)
  {
    frames = frames->FirstChildElement(frameNode.c_str());
    if (frames == 0)
    {
      cerr << "Incorrect xml structure" << endl;
      cerr << "Couldn't find Frame structure" << endl;
      return false;
    }
  }
  else
  {
    cerr << "Incorrect xml structure" << endl;
    cerr << "Couldn't find Frames structure" << endl;
    return false;
  }
  projectTitle = root->Attribute(projectName.c_str());
  imgFolderPath = root->Attribute(projectImgFolderPath.c_str());
  if (imgFolderPath.back() != '/')
  {
    imgFolderPath += '/';
  }
  maskFolderPath = root->Attribute(projectMaskFolderPath.c_str());
  if (maskFolderPath.back() != '/')
  {
     maskFolderPath += '/';
  }
  camFolderPath = root->Attribute(projectCamFolderPath.c_str());
  if (camFolderPath.back() != '/')
  {
    camFolderPath += '/';
  }
  allowScaling = root->BoolAttribute(projectAllowScaling.c_str());
  simMathPath = root->Attribute(projectSimMathPath.c_str());
  if (simMathPath.back() != '/')
  {
    simMathPath += '/';
  }
  exportPath = root->Attribute(projectExportPath.c_str());
  if (exportPath.back() != '/')
  {
    exportPath += '/';
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
  tree <BodyPart> trBodyParts, trBodyPartsCopy;
  tree <BodyPart>::iterator topBodyParts;
  topBodyParts = trBodyParts.begin();
  while (true)
  {
    if (bodyParts == 0) break;
    BodyPart part;
    XMLElement *e = bodyParts->ToElement();
    int parentJointId, childJointId;
    string name;
    int id;
    float expectedDistance;
    id = e->IntAttribute(bodyPartIdParam.c_str());
    name = e->Attribute(bodyPartNameParam.c_str());
    parentJointId = e->IntAttribute(bodyPartParentJointIdParam.c_str());
    childJointId = e->IntAttribute(bodyPartChildJointIdParam.c_str());
    expectedDistance = e->FloatAttribute(bodyPartExpectedDistanceParam.c_str());
    BodyJoint *parentJoint = 0, *childJoint = 0;
    for (topBodyJoints = trBodyJoints.begin(); topBodyJoints != trBodyJoints.end(); ++topBodyJoints)
    {
      if (parentJoint == 0 && topBodyJoints->getLimbID() == parentJointId)
      {
        parentJoint = &*topBodyJoints;
      }
      else if (childJoint == 0 && topBodyJoints->getLimbID() == childJointId)
      {
        childJoint = &*topBodyJoints;
      }
      if (parentJoint != 0 && childJoint != 0) break;
    }
    if (parentJoint == 0 || childJoint == 0)
    {
      cerr << "Could not find BodyJoint " << ((parentJoint == 0) ? parentJointId : childJointId) << endl;
      return false;
    }
    part.setPartID(id);
    part.setPartName(name);
    part.setParentJoint(parentJoint->getLimbID());
    part.setChildJoint(childJoint->getLimbID());
    part.setSpaceLength(expectedDistance);
    trBodyParts.insert(topBodyParts, part);
    bodyParts = bodyParts->NextSiblingElement();
  }
  while (true)
  {
    if (frames == 0) break;
    Frame *f = 0;
    XMLElement *e = frames->ToElement();
    string imgPath, maskPath, camPath;
    int id;
    bool isKeyFrame;
    Point2f gp;
    id = e->IntAttribute(frameIdParam.c_str());
    imgPath = e->Attribute(frameImgPathParam.c_str());
    maskPath = e->Attribute(frameMaskPathParam.c_str());
    camPath = e->Attribute(frameCamPathParam.c_str());
    isKeyFrame = e->BoolAttribute(frameIsKeyframeParam.c_str());
    gp.x = e->FloatAttribute(frameGPX.c_str());
    gp.y = e->FloatAttribute(frameGPY.c_str());
    if (isKeyFrame == true)
    {
      f = new Keyframe();
    }
    else
    {
      f = new Interpolation();
    }
    f->setID(id);
    Mat image = imread(curFolder + imgFolderPath + imgPath, CV_LOAD_IMAGE_COLOR);
    if (!image.data)
    {
      cerr << "Could not find file " << imgFolderPath + imgPath << endl;
      return false;
    }
    f->setImage(image);
    Mat mask = imread(curFolder + maskFolderPath + maskPath, CV_LOAD_IMAGE_GRAYSCALE);
    if (!mask.data)
    {
      cerr << "Could not find file " << maskFolderPath + maskPath << endl;
      return false;
    }
    f->setMask(mask);
    f->setGroundPoint(gp);
    PoseHelper::copyTree(trBodyJointsCopy, trBodyJoints);
    if (isKeyFrame)
    {
      bodyJoints = frames->FirstChildElement(bodyJointsNode.c_str());
      bodyJoints = bodyJoints->FirstChildElement(bodyJointNode.c_str());
      topBodyJoints = trBodyJointsCopy.begin();
      while (true)
      {
        if (bodyJoints == 0) break;
        XMLElement *e = bodyJoints->ToElement();
        int id;
        float x, y;
        bool depthSign;
        id = e->IntAttribute(bodyJointIdParam.c_str());
        x = e->FloatAttribute(bodyJointXParam.c_str());
        y = e->FloatAttribute(bodyJointYParam.c_str());
        depthSign = e->BoolAttribute(bodyJointDepthSignParam.c_str());
        BodyJoint *joint = 0;
        for (topBodyJoints = trBodyJointsCopy.begin(); topBodyJoints != trBodyJointsCopy.end(); ++topBodyJoints)
        {
          if (joint == 0 && topBodyJoints->getLimbID() == id)
          {
            joint = &*topBodyJoints;
          }
          if (joint != 0) break;
        }
        if (joint == 0)
        {
          cerr << "Could not find BodyJoint " << id;
          return false;
        }
        Point2f imgLocation = Point2f(x, y);
        joint->setImageLocation(imgLocation);
        joint->setDepthSign(depthSign);
        bodyJoints = bodyJoints->NextSiblingElement(); 
      }
    }
    PoseHelper::copyTree(trBodyPartsCopy, trBodyParts);
    if (isKeyFrame)
    {
      bodyParts = frames->FirstChildElement(bodyPartsNode.c_str());
      bodyParts = bodyParts->FirstChildElement(bodyPartNode.c_str());
      topBodyParts = trBodyPartsCopy.begin();
      while(true)
      {
        if (bodyParts == 0) break;
        XMLElement *e = bodyParts->ToElement();
        int id;
        bool isOccluded;
        id = e->IntAttribute(bodyPartIdParam.c_str());
        isOccluded = e->Attribute(bodyPartIsOccludedParam.c_str());
        BodyPart *part = 0;
        for (topBodyParts = trBodyPartsCopy.begin(); topBodyParts != trBodyPartsCopy.end(); ++topBodyParts)
        {
          if (part == 0 && topBodyParts->getPartID() == id)
          {
            part = &*topBodyParts;
          }
          if (part != 0) break;
        }
        if (part == 0)
        {
          cerr << "Could not find BodyPart " << id;
          return false;
        }
        part->setIsOccluded(isOccluded);
        bodyParts = bodyParts->NextSiblingElement();
      }
    }
    Skeleton skeleton;
    skeleton.setJointTree(trBodyJointsCopy);
    skeleton.setPartTree(trBodyPartsCopy);
//TODO (Vitaliy Koshura): This need to be loaded!!!!!!
    skeleton.setScale(100.0);    
    f->setSkeleton(skeleton);
    vFrames.push_back(f);
    frames = frames->NextSiblingElement();
  }
  return true;
}

bool ProjectLoader::Save(vector <vector <LimbLabel>> labels, string outFolder, int frameID)
{
  vector <vector <LimbLabel> >::iterator lls;
  vector <LimbLabel>::iterator ls; 
  for (lls = labels.begin(); lls != labels.end(); ++lls)
  {
    if (lls->size() == 0)
    {
      continue;
    }
    ofstream outFile;
    string outFileName = curFolder + outFolder;
    if (outFileName[outFileName.size()] != '/')
      outFileName += "/";
    stringstream ss;
    ss << frameID;
    ss << "-";
    ss << lls->begin()->getLimbID();
    outFileName += ss.str();
    outFile.open(outFileName);
    cerr << "Writing file: " << ss.str() << endl;
    for (ls = lls->begin(); ls != lls->end(); ++ls)
    {
      try
      {
        outFile << ls->getLimbID() << " ";
        outFile << ls->getCenter().x << " ";
        outFile << ls->getCenter().y << " ";
        outFile << ls->getAngle() << " ";
        if (ls->getPolygon().size() < 4)
        {
          outFile << "0" << " ";
        }
        else
        {
          outFile << PoseHelper::distSquared(ls->getPolygon()[0], ls->getPolygon()[2]) << " ";
        }
        outFile << ls->getSumScore() << " ";
        outFile << "0" << " ";
        outFile << "0" << " ";
        outFile << ((ls->getIsOccluded() == true) ? 0 : 1);
        outFile << std::endl;
      }
      catch(...)
      {
        cerr << "Empty LimbLabel" << endl;
      }
    }
    outFile.close();
  }
  return true;
}

