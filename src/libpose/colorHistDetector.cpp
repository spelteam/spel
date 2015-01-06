#include "colorHistDetector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "

//TODO (Vitaliy Koshura): Need unit test
ColorHistDetector::PartModel::PartModel(uint8_t _nBins) : nBins(_nBins)
{
  partHistogramm.resize(nBins);
  bgHistogramm.resize(nBins);
  for (uint8_t r = 0; r < nBins; r++)
  {
    partHistogramm[r].resize(nBins);
    bgHistogramm[r].resize(nBins);
    for (uint8_t g = 0; g < nBins; g++)
    {
      partHistogramm[r][g].resize(nBins);
      bgHistogramm[r][g].resize(nBins);
      for (int b = 0; b < nBins; b++)
      {
        partHistogramm[r][g][b] = 0.0;
        bgHistogramm[r][g][b] = 0.0;
      }
    }
  }
  sizeFG = 0;
}

ColorHistDetector::PartModel &ColorHistDetector::PartModel::operator=(PartModel &model)
{
  this->nBins = model.nBins;
  this->partHistogramm = model.partHistogramm;
  this->bgHistogramm = model.bgHistogramm;
  this->sizeFG = model.sizeFG;
  this->sizeBG = model.sizeBG;
  this->fgNumSamples = model.fgNumSamples;
  this->bgNumSamples = model.bgNumSamples;
  this->fgSampleSizes = model.fgSampleSizes;
  this->bgSampleSizes = model.bgSampleSizes;
  this->fgBlankSizes = model.fgBlankSizes;
  return *this;
}

//TODO (Vitaliy Koshura): Need unit test
ColorHistDetector::ColorHistDetector(uint8_t _nBins) : nBins(_nBins)
{
}

int ColorHistDetector::getID(void)
{
  return id;
}

void ColorHistDetector::setID(int _id)
{
  id = _id;
}

void ColorHistDetector::train(vector <Frame*> _frames, map <string, float> params)
{
  frames = _frames;
  sort(frames.begin(), frames.end(), FramePointerComparer());
  const float scaleParam = 1;
  const string sScaleParam = "scaleParam";
// first we need to check all used params
  params.emplace(sScaleParam, scaleParam);

  if (frames.size() == 0)
    throw logic_error("No input frames");
  partModels.clear();
// find skeleton from first keyframe or lockframe
  Skeleton skeleton;
  bool bFind = false;
  for (vector <Frame*>::iterator i = frames.begin(); i != frames.end(); ++i)
  {
    Frame *f = *i;
    if (f->getFrametype() == KEYFRAME || f->getFrametype() == LOCKFRAME)
    {
      skeleton = f->getSkeleton();
      bFind = true;
      break;
    }
  }
  if (bFind == false)
  {
#ifdef DEBUG
    cerr << ERROR_HEADER << "No neither keyframes nor lockframes" << endl;
#endif  // DEBUG
    throw logic_error("No neither keyframes nor lockframes");
  }
  tree <BodyPart> partTree;
  tree <BodyPart>::iterator iteratorBodyPart;
  for (vector <Frame*>::iterator frameNum = frames.begin(); frameNum != frames.end(); ++frameNum)
  {
    if ((*frameNum)->getFrametype() != KEYFRAME && (*frameNum)->getFrametype() != LOCKFRAME)
    {
      continue;
    }
#ifdef DEBUG
    cerr << "Training on frame " << (*frameNum)->getID() << endl;
#endif  // DEBUG
    map <int32_t, vector <Point3i>> partPixelColours;  // vector of vectors, by limb indeces
    map <int32_t, vector <Point3i>> bgPixelColours;
    map <int32_t, int> blankPixels;  // pixels outside the mask
    skeleton = (*frameNum)->getSkeleton();
    multimap <int32_t, POSERECT <Point2f>> polygons;  // polygons for this frame
    multimap <int32_t, bool> polyDepth;
    partTree = skeleton.getPartTree();
    tree <BodyJoint> jointTree;
    tree <BodyJoint>::iterator iteratorBodyJoint;
    for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
    {
      partPixelColours.insert(pair <int32_t, vector <Point3i>> (iteratorBodyPart->getPartID(), vector <Point3i>()));
      bgPixelColours.insert(pair <int32_t, vector <Point3i>> (iteratorBodyPart->getPartID(), vector <Point3i>()));
      blankPixels.insert(pair <int32_t, int> (iteratorBodyPart->getPartID(), 0));
      Point2f j1, j0;  // the two joint for this bone
      Point2f c1, c2, c3, c4;  // the four corners of the rectangle
      BodyJoint *joint = 0;
      joint = skeleton.getBodyJoint(iteratorBodyPart->getParentJoint());
      
      if (joint == 0)
      {
#ifdef DEBUG
        cerr << ERROR_HEADER << "Invalid parent joint" << endl;
#endif  // DEBUG
        break;
      }
      j0 = joint->getImageLocation();
      joint = 0;
      joint = skeleton.getBodyJoint(iteratorBodyPart->getChildJoint());
      if (joint == 0)
      {
#ifdef DEBUG
        cerr << ERROR_HEADER << "Invalid child joint" << endl;
#endif  // DEBUG
        break;
      }
      j1 = joint->getImageLocation();
      float boneLength = sqrt(PoseHelper::distSquared(j0, j1));
//TODO (Vitaliy Koshura): Check this!
      float boneWidth = 0;
      try
      {
        boneWidth = boneLength / iteratorBodyPart->getLWRatio()/*skeleton.getScale() * boneLength * params.at(sScaleParam)*/;
      }
      catch(...)
      {
        stringstream ss;
        ss << "Maybe there is no '" << sScaleParam << "' param";
#ifdef DEBUG
        cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
        throw logic_error(ss.str());
      }
      Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
      c1 = Point2f(0, 0.5 * boneWidth);
      c2 = Point2f(boneLength, 0.5 * boneWidth);
      c3 = Point2f(boneLength, -0.5 * boneWidth);
      c4 = Point2f(0, -0.5 * boneWidth);
      Point2f polyCenter = Point2f(boneLength * 0.5, 0);
      Point2f direction = j1 - j0;
      float rotationAngle = PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI);
// rotate polygon and translate
      c1 = PoseHelper::rotatePoint2D(c1, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c2 = PoseHelper::rotatePoint2D(c2, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c3 = PoseHelper::rotatePoint2D(c3, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c4 = PoseHelper::rotatePoint2D(c4, polyCenter, rotationAngle) + boxCenter - polyCenter;
      POSERECT <Point2f> poserect(c1, c2, c3, c4);
      polygons.insert(pair <int32_t, POSERECT <Point2f>> (iteratorBodyPart->getPartID(), poserect));
      polyDepth.insert(pair <int32_t, bool> (iteratorBodyPart->getPartID(), skeleton.getBodyJoint(iteratorBodyPart->getParentJoint())->getDepthSign()));
    }
    Mat maskMat = (*frameNum)->getMask();
    Mat imgMat = (*frameNum)->getImage();
    for (int32_t i = 0; i < imgMat.cols; i++)
    {
      for (int32_t j = 0; j < imgMat.rows; j++)
      {
        Vec3b intensity;
        try
        {
          intensity = imgMat.at<Vec3b>(j, i);
        }
        catch(...)
        {
          stringstream ss;
          ss << "Couldn't get imgMat value of indeces " << "[" << j << "][" << i << "]";
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          throw logic_error(ss.str());
        }
        uint8_t blue = intensity.val[0];
        uint8_t green = intensity.val[1];
        uint8_t red = intensity.val[2];
        uint8_t mintensity = 0;
        try
        {
          mintensity = maskMat.at<uint8_t>(j, i);
        }
        catch(...)
        {
          stringstream ss;
          ss << "Couldn't get maskMat value of indeces " << "[" << j << "][" << i << "]";
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          throw logic_error(ss.str());
        }
        bool blackPixel = mintensity < 10;
        int partHit = -1;
        bool depthSign = false;
        for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
        {
          uint32_t partNumber = iteratorBodyPart->getPartID();
          bool bContainsPoint = false;
          try
          {
            vector <POSERECT <Point2f>> partPolygons;
            multimap <int32_t, POSERECT <Point2f>>::iterator lower = polygons.lower_bound(partNumber), upper = polygons.upper_bound(partNumber);
            transform(lower, upper, back_inserter(partPolygons), [] (std::pair <int32_t, POSERECT<Point2f>> const &pair) { return pair.second; });
            for (vector <POSERECT <Point2f>>::iterator iteratorPartPolygons = partPolygons.begin(); iteratorPartPolygons != partPolygons.end(); ++iteratorPartPolygons)
            {
              if ((bContainsPoint = iteratorPartPolygons->containsPoint(Point2f(i, j)) > 0) == true)
              {
                break;
              }
            }
          }
          catch(...)
          {
            stringstream ss;
            ss << "There is no such polygon for body part " << partNumber;
#ifdef DEBUG
            cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
            throw logic_error(ss.str());
          }
          try
          {
            bool tempDepthSign;
            vector <bool> partDepthSigns;
            multimap <int32_t, bool>::iterator lower = polyDepth.lower_bound(partNumber), upper = polyDepth.upper_bound(partNumber);
            transform(lower, upper, back_inserter(partDepthSigns), [] (std::pair <int32_t, bool> const &pair) { return pair.second; });
            for (vector <bool>::iterator iteratorPartDepthSigns = partDepthSigns.begin(); iteratorPartDepthSigns != partDepthSigns.end(); ++iteratorPartDepthSigns)
            {
              if ((tempDepthSign = *iteratorPartDepthSigns) == true)
              {
                break;
              }
            }
            if (bContainsPoint && partHit == -1)
            {
              partHit = partNumber;
              depthSign = tempDepthSign;
            }
            else if (bContainsPoint && depthSign == false && tempDepthSign == true)
            {
              partHit = partNumber;
              depthSign = tempDepthSign;
            }
          }
          catch(...)
          {
            stringstream ss;
            ss << "There is no such polyDepth parameter for body part " << partNumber;
#ifdef DEBUG
            cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
            throw logic_error(ss.str());
          }

          partNumber++;
        }
        if (partHit != -1)
        {
          if (!blackPixel)
          {
            try
            {
              partPixelColours.at(partHit).push_back(Point3i(red, green, blue));
            }
            catch(...)
            {
              stringstream ss;
              ss << "There is no partPixelColours for body part " << partHit;
#ifdef DEBUG
              cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
              throw logic_error(ss.str());
            }
            for (tree <BodyPart>::iterator p = partTree.begin(); p != partTree.end(); ++p)
            {
              if ((int32_t)p->getPartID() != partHit)
              {
                try
                {
                  bgPixelColours.at(p->getPartID()).push_back(Point3i(red, green, blue));
                }
                catch(...)
                {
                  stringstream ss;
                  ss << "There is no such bgPixelColours for body part " << p->getPartID();
#ifdef DEBUG
                  cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
                  throw logic_error(ss.str());
                }
              }
            }
          }
          else
          {
            try
            {
              blankPixels.at(partHit)++;
            }
            catch(...)
            {
              stringstream ss;
              ss << "There is no such blankPixels for body part " << partHit;
#ifdef DEBUG
              cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
              throw logic_error(ss.str());
            }
          }
        }
        else
        {
          for (tree <BodyPart>::iterator p = partTree.begin(); p != partTree.end(); ++p)
          {
            try
            {
              bgPixelColours.at(p->getPartID()).push_back(Point3i(red, green, blue));
            }
            catch(...)
            {
              stringstream ss;
              ss << "There is no such bgPixelColours for body part " << p->getPartID();
#ifdef DEBUG
              cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
              throw logic_error(ss.str());
            }
          }
        }
      }
    }

// init partModels
    for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart) 
    {
      int32_t partNumber = iteratorBodyPart->getPartID();
      if (partModels.find(partNumber) == partModels.end())
      {
        PartModel model(nBins);
        partModels.insert(pair <int32_t, PartModel> (partNumber, model));
      }
      try
      {
        PartModel partModel = partModels.at(partNumber);
        vector <Point3i> partPixelColoursVector;
        vector <Point3i> bgPixelColoursVector;
        int blankPixelsCount;
        try
        {
          partPixelColoursVector = partPixelColours.at(partNumber);
        }
        catch(...)
        {
          stringstream ss;
          ss << "There is no such partPixelColours for body part " << partNumber;
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          throw logic_error(ss.str());
        }
        try
        {
          blankPixelsCount = blankPixels.at(partNumber);
        }
        catch(...)
        {
          stringstream ss;
          ss << "There is no such blankPixels for body part " << partNumber;
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          throw logic_error(ss.str());
        }
        try
        {
          bgPixelColoursVector = bgPixelColours.at(partNumber);
        }
        catch(...)
        {
          stringstream ss;
          ss << "There is no such bgPixelColours for body part " << partNumber;
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          throw logic_error(ss.str());
        }
        addPartHistogramm(partModel, partPixelColoursVector, blankPixelsCount);
        addBackgroundHistogramm(partModel, bgPixelColoursVector);
        partModels.at(partNumber) = partModel;
#ifdef DEBUG
        cerr << "Found part model: " << partNumber << endl;
#endif  // DEBUG
      }
      catch(...)
      {
        stringstream ss;
        ss << "Could not find part model " << partNumber;
#ifdef DEBUG
        cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
        throw logic_error(ss.str());
      }
      
    }
    maskMat.release();
    imgMat.release();
  }
}

vector <vector <LimbLabel> > ColorHistDetector::detect(Frame *frame, map <string, float> params)
{
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

  const float scaleParam = 1;
  const string sScaleParam = "scaleParam";

// first we need to check all used params
  params.emplace(sSearchDistCoeff, searchDistCoeff);
  params.emplace(sMinTheta, minTheta);
  params.emplace(sMaxTheta, maxTheta);
  params.emplace(sStepTheta, stepTheta);
  params.emplace(sUniqueLocationCandidates, uniqueLocationCandidates);
  params.emplace(sScaleParam, scaleParam);

  vector <vector <LimbLabel> > t;
  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();

  tree <BodyPart>::iterator iteratorBodyPart;
  map <int32_t, Mat> pixelDistributions = buildPixelDistributions(frame);
  map <int32_t, Mat> pixelLabels = buildPixelLabels(frame, pixelDistributions);
  Mat maskMat = frame->getMask();
  Frame *prevFrame = 0, *nextFrame = 0;
  uint32_t stepCount = 0;
  uint32_t step = 0;
  for (vector <Frame*>::iterator i = frames.begin(); i != frames.end(); ++i)
  {
    Frame *f = *i;
    if (f->getID() < frame->getID())
    {
      if (f->getFrametype() == KEYFRAME || f->getFrametype() == LOCKFRAME)
      {
        prevFrame = f;
        stepCount = 0;
      }
      else
      {
        stepCount++;
      }
    }
    else if (f->getID() > frame->getID())
    {
      stepCount++;
      if (f->getFrametype() == KEYFRAME || f->getFrametype() == LOCKFRAME)
      {
        nextFrame = f;
        break;
      }
    }
    else // equal
    {
      stepCount++;
      if (prevFrame == 0)
      {
        stringstream ss;
        ss << "Couldn't find previous keyframe to the frame " << frame->getID();
#ifdef DEBUG
        cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
        throw logic_error(ss.str());
      }
      else
      {
        if (stepCount == 0)
        {
          stringstream ss;
          ss << "Invalid stepCount";
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif // DEBUG
          throw logic_error(ss.str());
        }
        else
        {
          step = stepCount;
        }
      }
    }
  }
  if (prevFrame == 0)
  {
    stringstream ss;
    ss << "Couldn't find previous keyframe to the frame " << frame->getID();
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  if (nextFrame == 0)
  {
    stringstream ss;
    ss << "Couldn't find next feyframe to the frame " << frame->getID();
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    vector <LimbLabel> labels;
    BodyJoint *parentJoint = skeleton.getBodyJoint(iteratorBodyPart->getParentJoint());
    BodyJoint *childJoint = skeleton.getBodyJoint(iteratorBodyPart->getChildJoint());
    vector <Point2f> uniqueLocations;
    vector <LimbLabel> sortedLabels;
    vector <vector <LimbLabel>> allLabels;
    Skeleton prevSkeleton = prevFrame->getSkeleton();
    tree <BodyJoint> prevBodyJoints = prevSkeleton.getJointTree();
    Skeleton nextSkeleton = nextFrame->getSkeleton();
    tree <BodyJoint> nextBodyJoints = nextSkeleton.getJointTree();
    Point2f pj0, nj0, pj1, nj1;
    for (tree <BodyJoint>::iterator i = prevBodyJoints.begin(); i != prevBodyJoints.end(); ++i)
    {
      if (i->getLimbID() == parentJoint->getLimbID())
      {
        pj0 = i->getImageLocation();
      }
      if (i->getLimbID() == childJoint->getLimbID())
      {
        pj1 = i->getImageLocation();
      }
    }
    for (tree <BodyJoint>:: iterator i = nextBodyJoints.begin(); i != nextBodyJoints.end(); ++i)
    {
      if (i->getLimbID() == parentJoint->getLimbID())
      {
        nj0 = i->getImageLocation();
      }
      if (i->getLimbID() == childJoint->getLimbID())
      {
        nj1 = i->getImageLocation();
      }
    }
    float interpolateStep = (float)step / (float)stepCount;  // Interpolate2Ddisplacement
    Point2f j0 = pj0 * (1 - interpolateStep) + nj0 * interpolateStep;
    Point2f j1 = pj1 * (1 - interpolateStep) + nj1 * interpolateStep;
    float boneLength = (j0 == j1) ? 1.0 : sqrt(PoseHelper::distSquared(j0, j1));
    float boxWidth = 0;
    try
    {
      //TODO (Vitaliy Koshura): Need real implementation.
      boxWidth = boneLength / iteratorBodyPart->getLWRatio()/*skeleton.getScale() * boneLength * params.at(sScaleParam)*/;
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sScaleParam << "' param";
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
      throw logic_error(ss.str());
    }
    Point2f direction = j1 - j0;
    float theta = PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI);
    float minDist = boxWidth * 0.2;
    if (minDist < 2) minDist = 2;
    float searchDistance = 0;
    try
    {
      searchDistance = boneLength * params.at(sSearchDistCoeff);
    }
    catch(...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sSearchDistCoeff << "' param";
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
      throw logic_error(ss.str());
    }
    float minTheta = 0, maxTheta = 0, stepTheta = 0;
    try
    {
      minTheta = params.at(sMinTheta);
    }
    catch(...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sMinTheta << "' param";
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
      throw logic_error(ss.str());
    }
    try
    {
      maxTheta = params.at(sMaxTheta);
    }
    catch(...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sMaxTheta << "' param";
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
      throw logic_error(ss.str());
    }
     try
    {
      stepTheta = params.at(sStepTheta);
    }
    catch(...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sStepTheta << "' param";
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
      throw logic_error(ss.str());
    }
    Point2f suggestStart = 0.5 * j1 + 0.5 * j0;
    for (float x = suggestStart.x - searchDistance * 0.5; x < suggestStart.x + searchDistance * 0.5; x += minDist)
    {
      for (float y = suggestStart.y - searchDistance * 0.5; y < suggestStart.y + searchDistance * 0.5; y += minDist)
      {
        if (x < maskMat.cols && y < maskMat.rows)
        {
          uint8_t mintensity = 0;
          try
          {
            mintensity = maskMat.at<uint8_t>(y, x);
          }
          catch(...)
          {
            stringstream ss;
            ss << "Can't get value in maskMat at " << "[" << y << "][" << x << "]";
#ifdef DEBUG
            cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
            throw logic_error(ss.str());
          }
          bool blackPixel = mintensity < 10;
          if (!blackPixel)
          {
            for (float rot = theta - minTheta; rot < theta + maxTheta; rot += stepTheta)
            {
              Point2f p0 = Point2f(0, 0);
              Point2f p1 = Point2f(1.0, 0);
              p1 *= boneLength;
              p1 = PoseHelper::rotatePoint2D(p1, p0, rot);
              Point2f mid = 0.5 * p1;
              p1 = p1 + Point2f(x, y) - mid;
              p0 = Point2f(x, y) - mid;
              LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, frame, pixelDistributions, pixelLabels, p0, p1);
              sortedLabels.push_back(generatedLabel);
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
        LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, frame, pixelDistributions, pixelLabels, p0, p1);
        sortedLabels.push_back(generatedLabel);
      }
    }
    float uniqueLocationCandidates = 0;
    try
    {
      uniqueLocationCandidates = params.at(sUniqueLocationCandidates);
    }
    catch(...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sUniqueLocationCandidates << "' param";
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
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
          catch(...)
          {
            stringstream ss;
            ss << "There is no value of locations at " << "[" << i << "][" << j << "]";
#ifdef DEBUG
            cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
            throw logic_error(ss.str());
          }
        }
      }
      for (uint32_t i = 0; i < sortedLabels.size(); i++)
      {
        uint32_t x = sortedLabels.at(i).getCenter().x;
        uint32_t y = sortedLabels.at(i).getCenter().y;
        try
        {
          if (locations.at<uint32_t>(x, y) < uniqueLocationCandidates)
          {
            try
            {
              labels.push_back(sortedLabels.at(i));
            }
            catch(...)
            {
              stringstream ss;
              ss << "Maybe there is no value of sortedLabels at " << "[" << i << "]";
#ifdef DEBUG
              cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
              throw logic_error(ss.str());
            }
            locations.at<uint32_t>(x, y) += 1;
          }
        }
        catch(...)
        {
          stringstream ss;
          ss << "Maybe there is no value of locations at " << "[" << x << "][" << y << "]";
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          throw logic_error(ss.str());
        }
      }
      locations.release();
    }
    t.push_back(labels);
  }
  map <int32_t, Mat>::iterator i;
  for (i = pixelDistributions.begin(); i != pixelDistributions.end(); ++i)
  {
    i->second.release();
  }
  maskMat.release();
  return t;
}

uint8_t ColorHistDetector::getNBins(void)
{
  return nBins;
}

//TODO (Vitaliy Koshura): Need unit test
float ColorHistDetector::computePixelBelongingLikelihood(const PartModel &partModel, uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t factor = ceil(pow(2, 8)/partModel.nBins);
  float isFG = 0;
  try
  {
    isFG = partModel.partHistogramm.at(r/factor).at(g/factor).at(b/factor);
  }
  catch(...)
  {
    stringstream ss;
    ss << "Couldn't find partHistogramm " << "[" << (int)r/factor << "][" << (int)g/factor << "][" << (int)b/factor << "]";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  return isFG;
}

//TODO (Vitaliy Koshura>: need unit test
void ColorHistDetector::setPartHistogramm(PartModel &partModel, const vector <Point3i> &partColors)
{
  // do not add sample if the number of pixels is zero
  if (partColors.size() == 0)
    return;
  uint8_t factor = ceil(pow(2, 8)/partModel.nBins);  // divide the color space into bins
  partModel.sizeFG = partColors.size();
  partModel.fgNumSamples = 1;
  partModel.fgSampleSizes.clear();
  partModel.fgSampleSizes.push_back(partColors.size());

// clear histogram first
  for(uint8_t r = 0; r < partModel.nBins; r++)
  {
    for(uint8_t g = 0; g < partModel.nBins; g++)
    {
      for(uint8_t b = 0; b < partModel.nBins; b++)
      {
        try
        {
          partModel.partHistogramm.at(r).at(g).at(b) = 0.0;
        }
        catch(...)
        {
          stringstream ss;
          ss << "Couldn't find partHistogramm " << "[" << (int)r/factor << "][" << (int)g/factor << "][" << (int)b/factor << "]";
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          throw logic_error(ss.str());
        }
      }
    }
  }
  uint8_t r, g, b;
  for(uint32_t i = 0; i < partColors.size(); i++)
  {
    try
    {
      r = partColors.at(i).x / factor;
      g = partColors.at(i).y / factor;
      b = partColors.at(i).z / factor;
    }
    catch(...)
    {
      stringstream ss;
      ss << "Couldn't get partColors with index " << i;
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
      throw logic_error(ss.str());
    }
    try
    {
      partModel.partHistogramm.at(r).at(g).at(b)++;
    }
    catch(...)
    {
      stringstream ss;
      ss << "Couldn't find partHistogramm " << "[" << r << "][" << g << "][" << b << "]";
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
      throw logic_error(ss.str());
    }
  }
  for(uint8_t r = 0; r < partModel.nBins; r++)
  {
    for(uint8_t g = 0; g < partModel.nBins; g++)
    {
      for(uint8_t b = 0; b < partModel.nBins; b++)
      {
// normalise the histograms
        try
        {
          partModel.partHistogramm.at(r).at(g).at(b) /= partModel.sizeFG;
        }
        catch(...)
        {
          stringstream ss;
          ss << "Couldn't find partHistogramm " << "[" << r << "][" << g << "][" << b << "]";
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          throw logic_error(ss.str());
        }
      }
    }
  }
}

//TODO (Vitaliy Koshura>: need unit test
void ColorHistDetector::addPartHistogramm(PartModel &partModel, const vector <Point3i> &partColors, uint32_t nBlankPixels)
{
  if(partColors.size()==0) //do not add sample if the number of pixels is zero
    return;
//un-normalise
  for(uint8_t r = 0; r < partModel.nBins; r++)
  {
    for(uint8_t g = 0; g < partModel.nBins; g++)
    {
      for(uint8_t b = 0; b < partModel.nBins; b++)
      {
        partModel.partHistogramm[r][g][b] *= partModel.sizeFG;
      }
    }
  }

  int factor = ceil(pow(2, 8)/partModel.nBins);//divide the color space into bins
  partModel.sizeFG += partColors.size();
  partModel.fgNumSamples++;
  partModel.fgSampleSizes.push_back(partColors.size());

  for(uint32_t i = 0; i < partColors.size(); i++)
  {
    uint8_t r = partColors[i].x / factor;
    uint8_t g = partColors[i].y / factor;
    uint8_t b = partColors[i].z / factor;
    partModel.partHistogramm[r][g][b]++;
  }

//renormalise
  for(uint8_t r = 0; r < partModel.nBins; r++)
  {
    for(uint8_t g = 0; g < partModel.nBins; g++)
    {
      for(uint8_t b = 0; b < partModel.nBins; b++)
      {
//normalise the histograms
        partModel.partHistogramm[r][g][b] /= partModel.sizeFG;
      }
    }
  }

  partModel.fgBlankSizes.push_back(nBlankPixels); //add the number of blank pixels for this model
}

float ColorHistDetector::getAvgSampleSizeFg(const PartModel &partModel)
{
  float sum = 0;
  for(uint32_t i = 0; i < partModel.fgSampleSizes.size(); i++)
  {
    sum += partModel.fgSampleSizes[i];
  }
  sum /= partModel.fgNumSamples;
  return sum;
}

float ColorHistDetector::getAvgSampleSizeFgBetween(const PartModel &partModel, uint32_t s1, uint32_t s2)
{
  if(s1 >= partModel.fgSampleSizes.size() || s2 >= partModel.fgSampleSizes.size())
    return 0;
  return (partModel.fgSampleSizes[s1] + partModel.fgSampleSizes[s2]) / 2.0;
}

//TODO (Vitaliy Koshura): Need unit test
// Euclidean distance between part histograms
float ColorHistDetector::matchPartHistogramsED(const PartModel &partModelPrev, const PartModel &partModel) 
{
  float distance = 0;
  for(uint8_t r = 0; r < partModel.nBins; r++)
  {
    for(uint8_t g = 0; g < partModel.nBins; g++)
    {
      for(uint8_t b = 0; b < partModel.nBins; b++)
      {
        //normalise the histograms
        distance += pow(partModel.partHistogramm[r][g][b] - partModelPrev.partHistogramm[r][g][b], 2);
      }
    }
  }
  float score = sqrt(distance);
  return score;
}

void ColorHistDetector::addBackgroundHistogramm(PartModel &partModel, const vector <Point3i> &bgColors)
{
  if (bgColors.size() == 0)
    return;
// unnormalise
  for (uint8_t r = 0; r < partModel.nBins; r++)
  {
    for (uint8_t g = 0; g < partModel.nBins; g++)
    {
      for (uint8_t b = 0; b < partModel.nBins; b++)
      {
        partModel.bgHistogramm[r][g][b] *= partModel.sizeBG;
      }
    }
  }
  uint32_t factor = ceil(pow(2, 8) / partModel.nBins);
  partModel.sizeBG += bgColors.size();
  partModel.bgNumSamples++;
  partModel.bgSampleSizes.push_back(bgColors.size());
  for (uint32_t i = 0; i < bgColors.size(); i++)
  {
    partModel.bgHistogramm[bgColors[i].x / factor][bgColors[i].y / factor][bgColors[i].z / factor]++;
  }
// renormalise
  for (uint8_t r = 0; r < partModel.nBins; r++)
  {
    for (uint8_t g = 0; g < partModel.nBins; g++)
    {
      for (uint8_t b = 0; b < partModel.nBins; b++)
      {
        partModel.bgHistogramm[r][g][b] /= (float)partModel.sizeBG;
      }
    }
  }
}

map <int32_t, Mat> ColorHistDetector::buildPixelDistributions(Frame *frame)
{
  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;
  Mat imgMat = frame->getImage();
  Mat maskMat = frame->getMask();
  uint32_t width = imgMat.cols;
  uint32_t height = imgMat.rows;
  uint32_t mwidth = maskMat.cols;
  uint32_t mheight = maskMat.rows;
  map <int32_t, Mat> pixelDistributions;
  if (width != mwidth || height != mheight)
  {
    stringstream ss;
    ss << "Mask size not equal image size";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    Mat t = Mat(width, height, DataType <float>::type);
    int partID = iteratorBodyPart->getPartID();
    try
    {
      PartModel partModel = partModels.at(partID);
      for (uint32_t x = 0; x < width; x++)
      {
        for (uint32_t y = 0; y < height; y++)
        {
          Vec3b intensity = imgMat.at<Vec3b>(y, x);
          uint8_t blue = intensity.val[0];
          uint8_t green = intensity.val[1];
          uint8_t red = intensity.val[2];
          uint8_t mintensity = maskMat.at<uint8_t>(y, x);
          bool blackPixel = mintensity < 10;
  
          t.at<float>(x, y) = blackPixel ? 0 : computePixelBelongingLikelihood(partModel, red, green, blue);
        }
      }
    }
    catch(...)
    {
      stringstream ss;
      ss << "Maybe couldn't find partModel " << partID;
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
      throw logic_error(ss.str());
    }
    pixelDistributions.insert(pair <int32_t, Mat> (partID, t));
    t.release();
  }
  imgMat.release();
  maskMat.release();
  return pixelDistributions;
}

map <int32_t, Mat> ColorHistDetector::buildPixelLabels(Frame *frame, map <int32_t, Mat> pixelDistributions)
{
  Mat maskMat = frame->getMask();
  uint32_t width = maskMat.cols;
  uint32_t height = maskMat.rows;
  Skeleton skeleton = frame->getSkeleton();
  tree <BodyPart> partTree = skeleton.getPartTree();
  tree <BodyPart>::iterator iteratorBodyPart;
  map <int32_t, Mat> pixelLabels;
  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    Mat t = Mat(width, height, DataType <float>::type);
    Mat tt;
    try
    {
      tt = pixelDistributions.at(iteratorBodyPart->getPartID());
    }
    catch(...)
    {
      stringstream ss;
      ss << "Couldn't find distributions for body part " << iteratorBodyPart->getPartID();
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
      throw logic_error(ss.str());
    }
    for (uint32_t x = 0; x < width; x++)
    {
      for (uint32_t y = 0; y < height; y++)
      {
        uint8_t mintensity = maskMat.at<uint8_t>(y, x);
        bool blackPixel = mintensity < 10;
        if (!blackPixel)
        {
          float top = 0;
          float sum = 0;
          for (tree <BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
          {
            Mat temp;
            try
            {
              temp = pixelDistributions.at(i->getPartID());
            }
            catch(...)
            {
              stringstream ss;
              ss << "Couldn't find pixel distributions for body part " << i->getPartID();
#ifdef DEBUG
              cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
              throw logic_error(ss.str());
            }
            if (temp.at<float>(x, y) > top)
              top = temp.at<float>(x, y);
            sum += temp.at<float>(x, y);
            temp.release();
          }
          t.at<float>(x, y) = (top == 0) ? 0 : tt.at<float>(x, y) / (float)top;
        }
        else
        {
          t.at<float>(x, y) = 0;
        }
      }
    }
    pixelLabels.insert(pair<int32_t, Mat> (iteratorBodyPart->getPartID(), t));
    t.release();
  }
  maskMat.release();
  map <int32_t, Mat>::iterator i;
  for (i = pixelDistributions.begin(); i != pixelDistributions.end(); ++i)
  {
    i->second.release();
  }
  return pixelLabels;
}

LimbLabel ColorHistDetector::generateLabel(BodyPart bodyPart, Frame *frame, map <int32_t, Mat> pixelDistributions, map <int32_t, Mat> pixelLabels, Point2f j0, Point2f j1)
{
  vector <Score> s;
  Mat maskMat = frame->getMask();
  Mat imgMat = frame->getImage();
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
  float x = boxCenter.x;
  float y = boxCenter.y;
  float boneLength = sqrt(PoseHelper::distSquared(j0, j1));
  //TODO (Vitaliy Koshura): Need real implementation here
  float boxWidth = /*frame->getSkeleton().getScale() / */(boneLength / bodyPart.getLWRatio());
  float rot = PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI);
  vector <Point3i> partPixelColours;
  Point2f c1, c2, c3, c4, polyCenter;
  c1 = Point2f(0, 0.5 * boxWidth);
  c2 = Point2f(boneLength, 0.5 * boxWidth);
  c3 = Point2f(boneLength, -0.5 * boxWidth);
  c4 = Point2f(0, -0.5 * boxWidth);
  polyCenter = Point2f(boneLength * 0.5, 0);
  c1 = PoseHelper::rotatePoint2D(c1, polyCenter, rot) + boxCenter - polyCenter;
  c2 = PoseHelper::rotatePoint2D(c2, polyCenter, rot) + boxCenter - polyCenter;
  c3 = PoseHelper::rotatePoint2D(c3, polyCenter, rot) + boxCenter - polyCenter;
  c4 = PoseHelper::rotatePoint2D(c4, polyCenter, rot) + boxCenter - polyCenter;
  POSERECT <Point2f> rect(c1, c2, c3, c4);
  uint32_t totalPixels = 0;
  uint32_t pixelsInMask = 0;
  uint32_t pixelsWithLabel = 0;
  float totalPixelLabelScore = 0;
  float pixDistAvg = 0;
  float pixDistNum = 0;
  PartModel model;
  stringstream detectorName;
  detectorName << getID();
  try
  {
    model = partModels.at(bodyPart.getPartID());
  }
  catch(...)
  {
    stringstream ss;
    ss << "Couldn't get partModel of bodyPart " << bodyPart.getPartID();
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  if (getAvgSampleSizeFg(model) == 0)
  {
    maskMat.release();
    imgMat.release();
    stringstream ss;
    ss << "getAvgSampleSizeFg(model) == 0";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif
    throw logic_error(ss.str());
  }
  for (int32_t i = x - boneLength * 0.5; i < x + boneLength * 0.5; i++)
  {
    for (int32_t j = y - boneLength * 0.5; j < y + boneLength * 0.5; j++)
    {
      if (i < maskMat.cols && j < maskMat.rows)
      {
        float xmax, ymax, xmin, ymin;
        rect.GetMinMaxXY <float> (xmin, ymin, xmax, ymax);

        if (i <= xmax && i >= xmin && j <= ymax && j >= ymin)
        {
          if (rect.containsPoint(Point2f(i,j)) > 0)
          {
            totalPixels++;
            uint8_t mintensity = 0;
            try
            {
              mintensity = maskMat.at<uint8_t>(j, i);
            }
            catch(...)
            {
              stringstream ss;
              ss << "Couldn't get value of maskMat at " << "[" << j << "][" << i << "]";
#ifdef DEBUG
              cerr << ERROR_HEADER << ss.str() << endl;
#endif // DEBUG
              throw logic_error(ss.str());
            }
            bool blackPixel = mintensity < 10;
            if (!blackPixel)
            {
              try
              {
                pixDistAvg += pixelDistributions.at(bodyPart.getPartID()).at<float>(i, j);
              }
              catch(...)
              {
                stringstream ss;
                ss << "Couldn't find pixel distribution for body part " << bodyPart.getPartID();
#ifdef DEBUG
                cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
                throw logic_error(ss.str());
              }
              pixDistNum++;
              try
              {
                if (pixelLabels.at(bodyPart.getPartID()).at<float>(i, j))
                {
                  pixelsWithLabel++;
                  totalPixelLabelScore += pixelLabels.at(bodyPart.getPartID()).at<float>(i, j);
                }
              }
              catch(...)
              {
                stringstream ss;
                ss << "Couldn't find pixel labels for body part " << bodyPart.getPartID();
#ifdef DEBUG
                cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
                throw logic_error(ss.str());
              }
              Vec3b intensity = imgMat.at<Vec3b>(j, i);
              uint8_t blue = intensity.val[0];
              uint8_t green = intensity.val[1];
              uint8_t red = intensity.val[2];
              Point3i ptColor(red, green, blue);
              pixelsInMask++;
              partPixelColours.push_back(ptColor);
            }
          }
        }
      }
    }
  }
  maskMat.release();
  imgMat.release();
  float supportScore = 0;
  float inMaskSupportScore = 0;
  pixDistAvg /= (float)pixDistNum;
  if (partPixelColours.size() > 0)
  {
    supportScore = (float)totalPixelLabelScore / (float)totalPixels;
    inMaskSupportScore = (float)totalPixelLabelScore / (float)pixelsInMask;
    PartModel model(nBins);
    setPartHistogramm(model, partPixelColours);
    float score = 1.0 - (supportScore + inMaskSupportScore);
    /*if (score < 0)
    {
      stringstream ss;
      ss << "Score can't be less thah zero" << endl;
#ifdef DEBUG
      cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
      throw logic_error(ss.str());
    }*/
    Score sc(score, detectorName.str());
    s.push_back(sc);
    return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s);
  }
  cerr << "Dirty label!" << endl;
  Score sc(1.0, detectorName.str());
  s.push_back(sc);
  return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s);
}

