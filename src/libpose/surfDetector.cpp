#include "surfDetector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "

SurfDetector::SurfDetector(void)
{
  id = 0x5344;
#if OpenCV_VERSION_MAJOR == 2 && OpenCV_VERSION_MINOR == 4 && OpenCV_VERSION_PATCH >= 9
  initModule_nonfree();
#endif
}

int SurfDetector::getID(void)
{
  return id;
}

void SurfDetector::setID(int _id)
{
  id = _id;
}

//TODO (Vitaliy Koshura): Write real implementation here
void SurfDetector::train(vector <Frame*> _frames, map <string, float> params)
{
  frames = _frames;

  const uint8_t debugLevel = 1;
  const string sDebugLevel = "debugLevel";
  const uint32_t minHessian = 500;
  const string sMinHessian = "minHessian";

  params.emplace(sDebugLevel, debugLevel);
  params.emplace(sMinHessian, minHessian);

  debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

  for (vector <Frame*>::iterator frameNum = frames.begin(); frameNum != frames.end(); ++frameNum)
  {
    if ((*frameNum)->getFrametype() != KEYFRAME && (*frameNum)->getFrametype() != LOCKFRAME)
    {
      continue;
    }

    if (debugLevelParam >= 2)
      cerr << "Training on frame " << (*frameNum)->getID() << endl;

    try
    {
      partModels.insert(pair <uint32_t, map <uint32_t, PartModel>>((*frameNum)->getID(), computeDescriptors(*frameNum, minHessian)));
    }
    catch (...)
    {
      break;
    }
  }

}

//TODO (Vitaliy Koshura): Write real implementation here
vector <vector <LimbLabel> > SurfDetector::detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
{
  const uint32_t minHessian = 500;
  const string sMinHessian = "minHessian";

  const float searchDistCoeff = 0.5;
  const string sSearchDistCoeff = "searchDistCoeff";

  const float minTheta = 90;
  const string sMinTheta = "minTheta";

  const float maxTheta = 100;
  const string sMaxTheta = "maxTheta";

  const float stepTheta = 10;
  const string sStepTheta = "stepTheta";

  const uint32_t uniqueLocationCandidates = 4;
  const string sUniqueLocationCandidates = "uniqueLocationCandidates";

  //const float scaleParam = 1;
  //const string sScaleParam = "scaleParam";

  const float searchDistCoeffMult = 1.25;
  const string sSearchDistCoeffMult = "searchDistCoeffMult";

  const float useSURFdet = 1.0f;
  const string sUseSURFdet = "useSURFdet";

  const uint8_t debugLevel = 1;
  const string sDebugLevel = "debugLevel";

  const float rotationThreshold = 0.025f;
  const string sRotationThreshold = "rotationThreshold";

  // first we need to check all used params
  params.emplace(sMinHessian, minHessian);
  params.emplace(sSearchDistCoeff, searchDistCoeff);
  params.emplace(sMinTheta, minTheta);
  params.emplace(sMaxTheta, maxTheta);
  params.emplace(sStepTheta, stepTheta);
  params.emplace(sUniqueLocationCandidates, uniqueLocationCandidates);
  //params.emplace(sScaleParam, scaleParam);
  params.emplace(sSearchDistCoeffMult, searchDistCoeffMult);
  params.emplace(sUseSURFdet, useSURFdet);
  params.emplace(sDebugLevel, debugLevel);
  params.emplace(sRotationThreshold, rotationThreshold);

  debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

  vector <vector <LimbLabel> > t;

  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;

  Mat maskMat = frame->getMask();

  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    vector <LimbLabel> labels;
    vector <Point2f> uniqueLocations;
    vector <LimbLabel> sortedLabels;
    vector <vector <LimbLabel>> allLabels;
    Point2f j0, j1;

    try
    {
      j0 = skeleton.getBodyJoint(iteratorBodyPart->getParentJoint())->getImageLocation();
      j1 = skeleton.getBodyJoint(iteratorBodyPart->getChildJoint())->getImageLocation();
    }
    catch (...)
    {
      stringstream ss;
      ss << "Can't get joints";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }

    float boneLength = getBoneLength(j0, j1);
    float boxWidth = getBoneWidth(boneLength, *iteratorBodyPart);
    Point2f direction = j1 - j0;
    float theta = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));
    float minDist = boxWidth * 0.2f;
    if (minDist < 2) minDist = 2;
    float searchDistance = iteratorBodyPart->getSearchRadius();
    try
    {
      if (searchDistance <= 0)
        searchDistance = boneLength * params.at(sSearchDistCoeff);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sSearchDistCoeff << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    float minTheta = 0, maxTheta = 0, stepTheta = 0;
    try
    {
      minTheta = params.at(sMinTheta);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sMinTheta << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    try
    {
      maxTheta = params.at(sMaxTheta);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sMaxTheta << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    try
    {
      stepTheta = params.at(sStepTheta);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sStepTheta << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    Point2f suggestStart = 0.5 * j1 + 0.5 * j0;
    for (float x = suggestStart.x - searchDistance * 0.5f; x < suggestStart.x + searchDistance * 0.5f; x += minDist)
    {
      for (float y = suggestStart.y - searchDistance * 0.5f; y < suggestStart.y + searchDistance * 0.5f; y += minDist)
      {
        if (x < maskMat.cols && y < maskMat.rows)
        {
          uint8_t mintensity = 0;
          try
          {
            mintensity = maskMat.at<uint8_t>((int)y, (int)x);
          }
          catch (...)
          {
            stringstream ss;
            ss << "Can't get value in maskMat at " << "[" << y << "][" << x << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
          bool blackPixel = mintensity < 10;
          if (!blackPixel)
          {
            for (float rot = theta - minTheta; rot < theta + maxTheta; rot += stepTheta)
            {
              if (abs(rot) < abs(iteratorBodyPart->getRotationSearchRange()) + abs(rotationThreshold))
              {
                Point2f p0 = Point2f(0, 0);
                Point2f p1 = Point2f(1.0, 0);
                p1 *= boneLength;
                p1 = PoseHelper::rotatePoint2D(p1, p0, rot);
                Point2f mid = 0.5 * p1;
                p1 = p1 + Point2f(x, y) - mid;
                p0 = Point2f(x, y) - mid;               
                LimbLabel generatedLabel = generateLabel(frame, *iteratorBodyPart, p0, p1, computeDescriptors(*iteratorBodyPart, p0, p1, frame->getImage(), minHessian), useSURFdet);
                sortedLabels.push_back(generatedLabel);
              }
            }
          }
        }
      }
    }
    if (sortedLabels.size() == 0)
    {
      for (float rot = theta - minTheta; rot < theta + maxTheta; rot += stepTheta)
      {
        Point2f p0 = Point2f(0, 0);
        Point2f p1 = Point2f(1.0, 0);
        p1 *= boneLength;
        p1 = PoseHelper::rotatePoint2D(p1, p0, rot);
        Point2f mid = 0.5 * p1;
        p1 = p1 + Point2f(suggestStart.x, suggestStart.y) - mid;
        p0 = Point2f(suggestStart.x, suggestStart.y) - mid;       
        LimbLabel generatedLabel = generateLabel(frame, *iteratorBodyPart, p0, p1, computeDescriptors(*iteratorBodyPart, p0, p1, frame->getImage(), minHessian), useSURFdet);
        sortedLabels.push_back(generatedLabel);
      }
    }
    float uniqueLocationCandidates = 0;
    try
    {
      uniqueLocationCandidates = params.at(sUniqueLocationCandidates);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sUniqueLocationCandidates << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    if (sortedLabels.size() > 0)
    {
      sort(sortedLabels.begin(), sortedLabels.end());
      Mat locations(frame->getImage().cols, frame->getImage().rows, DataType<uint32_t>::type);
      for (int32_t i = 0; i < frame->getImage().cols; i++)
      {
        for (int32_t j = 0; j < frame->getImage().rows; j++)
        {
          try
          {
            locations.at<uint32_t>(i, j) = 0;
          }
          catch (...)
          {
            stringstream ss;
            ss << "There is no value of locations at " << "[" << i << "][" << j << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
      }
      for (uint32_t i = 0; i < sortedLabels.size(); i++)
      {
        uint32_t x = (uint32_t)sortedLabels.at(i).getCenter().x;
        uint32_t y = (uint32_t)sortedLabels.at(i).getCenter().y;
        try
        {
          if (locations.at<uint32_t>(x, y) < uniqueLocationCandidates)
          {
            try
            {
              labels.push_back(sortedLabels.at(i));
            }
            catch (...)
            {
              stringstream ss;
              ss << "Maybe there is no value of sortedLabels at " << "[" << i << "]";
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
            locations.at<uint32_t>(x, y) += 1;
          }
        }
        catch (...)
        {
          stringstream ss;
          ss << "Maybe there is no value of locations at " << "[" << x << "][" << y << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
      }
      locations.release();
    }
    t.push_back(labels);
  }
  return merge(limbLabels, t);
}

map <uint32_t, SurfDetector::PartModel> SurfDetector::computeDescriptors(Frame *frame, uint32_t minHessian)
{
  map <uint32_t, PartModel> parts;
  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  Mat imgMat = frame->getImage();
  for (tree <BodyPart>::iterator part = partTree.begin(); part != partTree.end(); ++part)
  {
    Point2f j0, j1;
    BodyJoint *joint = skeleton.getBodyJoint(part->getParentJoint());
    if (joint == 0)
    {
      stringstream ss;
      ss << "Invalid parent joint";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    j0 = joint->getImageLocation();
    joint = 0;
    joint = skeleton.getBodyJoint(part->getChildJoint());
    if (joint == 0)
    {
      stringstream ss;
      ss << "Invalid child joint";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    j1 = joint->getImageLocation();
    Point2f direction = j1 - j0; // used as estimation of the vector's direction
    float rotationAngle = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI)); //bodypart tilt angle 
    part->setRotationSearchRange(rotationAngle);
    try
    {
      parts.insert(pair <uint32_t, PartModel>(part->getPartID(), computeDescriptors(*part, j0, j1, imgMat, minHessian)));
    }
    catch (logic_error err)
    {
      stringstream ss;
      ss << "Can't compute descriptors for the frame " << frame->getID() << " for the part " << part->getPartID() << endl;
      ss << "\t" << err.what();
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
  }
  skeleton.setPartTree(partTree);
  frame->setSkeleton(skeleton);
  return parts;
}

SurfDetector::PartModel SurfDetector::computeDescriptors(BodyPart bodyPart, Point2f j0, Point2f j1, Mat imgMat, uint32_t minHessian)
{
  float boneLength = getBoneLength(j0, j1);
  float boneWidth = getBoneWidth(boneLength, bodyPart);
  Size originalSize = Size(static_cast <uint32_t> (boneLength), static_cast <uint32_t> (boneWidth));
  POSERECT <Point2f> rect = getBodyPartRect(bodyPart, j0, j1, originalSize);

  float xmax, ymax, xmin, ymin;
  rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
  Point2f direction = j1 - j0;
  float rotationAngle = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));
  PartModel partModel;
  partModel.partModelRect = rect;
  Mat partImage = rotateImageToDefault(imgMat, partModel.partModelRect, rotationAngle, originalSize);

#if OpenCV_VERSION_MAJOR == 3
  Ptr <SurfFeatureDetector> detector = SurfFeatureDetector::create(minHessian);
  detector->detect(partImage, partModel.keyPoints);
  Ptr <SurfDescriptorExtractor> extractor = SurfDescriptorExtractor::create();
  extractor->compute(partImage, partModel.keyPoints, partModel.descriptors);
#else
  SurfFeatureDetector detector(minHessian);
  detector.detect(partImage, partModel.keyPoints);
  SurfDescriptorExtractor extractor;
  extractor.compute(partImage, partModel.keyPoints, partModel.descriptors);
#endif
  return partModel;
}

LimbLabel SurfDetector::generateLabel(Frame *frame, BodyPart bodyPart, Point2f j0, Point2f j1, PartModel partModel, float _useSURFdet)
{
  vector <Score> s;
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
  float rot = float(PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
  POSERECT <Point2f> rect = getBodyPartRect(bodyPart, j0, j1);
  stringstream detectorName;
  detectorName << getID();

  uint32_t totalPixels = 0;
  uint32_t inMaskPixels = 0;
  float boneLength = getBoneLength(j0, j1); // distance between joints
  Mat imgMat = frame->getImage(); // copy image from the frame
  Mat maskMat = frame->getMask(); // copy mask from the frame 
  float xmax, ymax, xmin, ymin;
  rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax); // highlight the extreme points of the body part rect

  // Scan the area near the bodypart center
  for (int32_t i = int32_t(boxCenter.x - boneLength * 0.5); i < int32_t(boxCenter.x + boneLength * 0.5); i++)
  {
    for (int32_t j = int32_t(boxCenter.y - boneLength * 0.5); j < int32_t(boxCenter.y + boneLength * 0.5); j++)
    {
      if (i < maskMat.cols && j < maskMat.rows) // if the point is within the image
      {
        if (i <= xmax && i >= xmin && j <= ymax && j >= ymin) // if the point within the highlight area
        {
          if (rect.containsPoint(Point2f((float)i, (float)j)) > 0) // if the point belongs to the rectangle
          {
            totalPixels++; // counting of the contained pixels
            uint8_t mintensity = 0;
            try
            {
              mintensity = maskMat.at<uint8_t>(j, i); // copy current point mask value 
            }
            catch (...)
            {
              maskMat.release();
              imgMat.release();
              if (debugLevelParam >= 2)
                cerr << ERROR_HEADER << "Dirty label!" << endl;
              Score sc(-1.0f, detectorName.str(), _useSURFdet);
              s.push_back(sc);
              return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s, true); // create the limb label
            }
            bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
            if (!blackPixel)
            {
              inMaskPixels++; // counting of the all scanned pixels
            }
          }
        }
      }
    }
  }
  maskMat.release();
  imgMat.release();
  float score = compare(bodyPart, partModel);
  score += inMaskPixels / totalPixels;
  Score sc(score, detectorName.str(), _useSURFdet);
  s.push_back(sc);
  return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s);
}

float SurfDetector::compare(BodyPart bodyPart, PartModel model)
{
  float score = 0;
  uint32_t count = 0;
  FlannBasedMatcher matcher;
  vector <vector <DMatch>> matches;

  for (map <uint32_t, map <uint32_t, PartModel>>::iterator framePartModels = partModels.begin(); framePartModels != partModels.end(); ++framePartModels)
  {
    count++;
    for (map <uint32_t, PartModel>::iterator partModel = framePartModels->second.begin(); partModel != framePartModels->second.end(); ++partModel)
    {
      if (partModel->first != static_cast <uint32_t> (bodyPart.getPartID()))
      {
        continue;
      }
      else
      {
        matcher.knnMatch(partModel->second.descriptors, model.descriptors, matches, 2);
        float s = 0;
        for (uint32_t i = 0; i < matches.size(); i++)
        {
          if ((matches[i][0].distance < 0.6*(matches[i][1].distance)) && ((int)matches[i].size() <= 2 && (int)matches[i].size()>0))
          {
            s += matches[i][0].distance;
          }
        }
        score += s / matches.size();
        break;
      }
    }
  }
  return score /= count;
}
