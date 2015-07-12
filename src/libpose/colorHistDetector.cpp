#include "colorHistDetector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "

// PartModel Constructor 
// Initialization "partHistogramm" and "bgHistogramm" with _nBins^3 elements capacity 3-D arrays 
ColorHistDetector::PartModel::PartModel(uint8_t _nBins) : nBins(_nBins)
{
  partHistogramm.resize(nBins);
  bgHistogramm.resize(nBins);
  for (uint8_t r = 0; r < nBins; r++)
  {
    try
    {
      partHistogramm.at(r).resize(nBins);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't find partHistogramm " << "[" << r << "]";
      throw logic_error(ss.str());
    }
    try
    {
      bgHistogramm.at(r).resize(nBins);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't find bgHistogramm " << "[" << r << "]";
      throw logic_error(ss.str());
    }
    for (uint8_t g = 0; g < nBins; g++)
    {
      try
      {
        partHistogramm.at(r).at(g).resize(nBins);
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't find partHistogramm " << "[" << r << "][" << g << "]";
        throw logic_error(ss.str());
      }
      try
      {
        bgHistogramm.at(r).at(g).resize(nBins);
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't find bgHistogramm " << "[" << r << "][" << g << "]";
        throw logic_error(ss.str());
      }
      for (int b = 0; b < nBins; b++)
      {
        try
        {
          partHistogramm.at(r).at(g).at(b) = 0.0;
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't find partHistogramm " << "[" << r << "][" << g << "][" << b << "]";
          throw logic_error(ss.str());
        }
        try
        {
          bgHistogramm.at(r).at(g).at(b) = 0.0;
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't find bgHistogramm " << "[" << r << "][" << g << "][" << b << "]";
          throw logic_error(ss.str());
        }
      }
    }
  }
  sizeFG = 0;
}

// Copy all fields of the "PartModel" structure
ColorHistDetector::PartModel &ColorHistDetector::PartModel::operator=(const PartModel &model)
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

// Constructor with initialization of constant field "nBins"
ColorHistDetector::ColorHistDetector(uint8_t _nBins) : nBins(_nBins)
{
  id = 0x434844;
}

// Returns unique ID of "ColorHistDetector" object
int ColorHistDetector::getID(void) const
{
  return id;
}

// Change ID of "ColorHistDetector" object
void ColorHistDetector::setID(int _id)
{
  id = _id;
}

// Builds a histograms of all polygons for pre-marked frames
void ColorHistDetector::train(vector <Frame*> _frames, map <string, float> params)
{
  frames = _frames; // vector of pointers - presents a sequence of frames
  sort(frames.begin(), frames.end(), FramePointerComparer()); // sorting frames by id
  //const float scaleParam = 1; // scaling coefficient
  //const string sScaleParam = "scaleParam";
#ifdef DEBUG
  const uint8_t debugLevel = 5;
#else
  const uint8_t debugLevel = 1;
#endif // DEBUG
  const string sDebugLevel = "debugLevel";
  // first we need to check all used params
  //params.emplace(sScaleParam, scaleParam);
  params.emplace(sDebugLevel, debugLevel);

  debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

  if (frames.size() == 0)
    throw logic_error("No input frames"); // the sequence of frames is empty
  partModels.clear();
  // Find skeleton from first keyframe or lockframe
  Skeleton skeleton;

  const string sMaxFrameHeight = "maxFrameHeight";

  params.emplace(sMaxFrameHeight, frames.at(0)->getFrameSize().height);

  maxFrameHeight = params.at(sMaxFrameHeight);

  bool bFind = false; // flag, indicate the presence of marked frame in the sequence
  for (vector <Frame*>::iterator i = frames.begin(); i != frames.end(); ++i)
  {
    Frame *f = *i;
    if (f->getFrametype() == KEYFRAME || f->getFrametype() == LOCKFRAME)
    {
      skeleton = f->getSkeleton();
      bFind = true; // marked frame was found
      break;
    }
  }
  if (bFind == false)
  {
    if (debugLevelParam >= 1)
      cerr << ERROR_HEADER << "No neither keyframes nor lockframes" << endl;
    throw logic_error("No neither keyframes nor lockframes");
  }

  tree <BodyPart> partTree;
  // Handling all frames
  for (vector <Frame*>::iterator frameNum = frames.begin(); frameNum != frames.end(); ++frameNum)
  {
    if ((*frameNum)->getFrametype() != KEYFRAME && (*frameNum)->getFrametype() != LOCKFRAME)
    {
      continue; // skip unmarked frames
    }

    int originalSize = (*frameNum)->getFrameSize().height;

    Frame *workFrame = 0;
    if ((*frameNum)->getFrametype() == KEYFRAME)
      workFrame = new Keyframe();
    else if ((*frameNum)->getFrametype() == LOCKFRAME)
      workFrame = new Lockframe();
    else if ((*frameNum)->getFrametype() == INTERPOLATIONFRAME)
      workFrame = new Interpolation();

    workFrame = (*frameNum)->clone(workFrame);

    workFrame->Resize(params.at(sMaxFrameHeight));

    if (debugLevelParam >= 2)
      cerr << "Training on frame " << workFrame->getID() << endl;
    // Create local variables
    map <int32_t, vector <Point3i>> partPixelColours; // the set of RGB-colours of pixel's for current body part
    map <int32_t, vector <Point3i>> bgPixelColours; // the set of RGB-colours for a pixels of background
    map <int32_t, int> blankPixels;  // pixels outside the mask
    skeleton = workFrame->getSkeleton(); // copy marking from current frame
    multimap <int32_t, POSERECT <Point2f>> polygons;  // polygons for this frame
    multimap <int32_t, float> polyDepth; // used for evaluation of overlapped polygons
    partTree = skeleton.getPartTree(); // the skeleton body parts
    // Handling all bodyparts on the frames
    for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
    {
      partPixelColours.insert(pair <int32_t, vector <Point3i>>(iteratorBodyPart->getPartID(), vector <Point3i>())); // container initialization for conserve colours set for each of body parts
      bgPixelColours.insert(pair <int32_t, vector <Point3i>>(iteratorBodyPart->getPartID(), vector <Point3i>())); // container initialization for conserve background colours set for each of body parts
      blankPixels.insert(pair <int32_t, int>(iteratorBodyPart->getPartID(), 0)); // container initialization for counting blank pixels for each of body parts
      Point2f j1, j0;  // temporary adjacent joints   
      BodyJoint *joint = 0; // temporary conserve a joints
      joint = skeleton.getBodyJoint(iteratorBodyPart->getParentJoint()); // the parent node of current body part pointer 

      if (joint == 0)
      {
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << "Invalid parent joint" << endl;
        break; // a joint has no marking on the frame
      }
      j0 = joint->getImageLocation(); // coordinates of current joint
      joint = 0;
      joint = skeleton.getBodyJoint(iteratorBodyPart->getChildJoint()); // the child node of current body part pointer
      if (joint == 0)
      {
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << "Invalid child joint" << endl;
        break; // a joint has no marking on the frame
      }
      j1 = joint->getImageLocation(); // coordinates of current joint
      float boneLength = getBoneLength(j0, j1); // distance between nodes
      //TODO (Vitaliy Koshura): Check this!
      float boneWidth;
      try
      { //currents body part polygon width 
        boneWidth = getBoneWidth(boneLength, *iteratorBodyPart);
      }
      catch (...)
      {
        stringstream ss;
        ss << "Can't get LWRatio value";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
      Point2f direction = j1 - j0; // used as estimation of the vector's direction
      float rotationAngle = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI)); //bodypart tilt angle 
      iteratorBodyPart->setRotationSearchRange(rotationAngle);
      POSERECT <Point2f> poserect = getBodyPartRect(*iteratorBodyPart, j0, j1);
      polygons.insert(pair <int32_t, POSERECT <Point2f>>(iteratorBodyPart->getPartID(), poserect));
      polyDepth.insert(pair <int32_t, float>(iteratorBodyPart->getPartID(), skeleton.getBodyJoint(iteratorBodyPart->getParentJoint())->getSpaceLocation().z));
    }
    skeleton.setPartTree(partTree);
    workFrame->setSkeleton(skeleton);
    Mat maskMat = workFrame->getMask(); // copy mask from the current frame
    Mat imgMat = workFrame->getImage(); // copy image from the current frame
    // Range over all pixels of the frame
    for (int32_t i = 0; i < imgMat.cols; i++)
    {
      for (int32_t j = 0; j < imgMat.rows; j++)
      {
        Vec3b intensity;
        try
        {
          intensity = imgMat.at<Vec3b>(j, i);  // copy RGB color of current pixel
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't get imgMat value of indeces " << "[" << j << "][" << i << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
        // Copy the current pixel colour components
        uint8_t blue = intensity.val[0];
        uint8_t green = intensity.val[1];
        uint8_t red = intensity.val[2];
        uint8_t mintensity = 0;
        try
        {
          mintensity = maskMat.at<uint8_t>(j, i);  // copy current pixel mask value 
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't get maskMat value of indeces " << "[" << j << "][" << i << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
        bool blackPixel = mintensity < 10;
        int partHit = -1; // will be equal to -1 until is not found polygon, which contains the point
        float depth = 0;
        // Handling all poligons
        for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
        {
          uint32_t partNumber = iteratorBodyPart->getPartID();
          bool bContainsPoint = false;
          try
          {
            vector <POSERECT <Point2f>> partPolygons;
            // Copy poligons to "PartPoligons"
            multimap <int32_t, POSERECT <Point2f>>::iterator lower = polygons.lower_bound(partNumber), upper = polygons.upper_bound(partNumber);
            transform(lower, upper, back_inserter(partPolygons), [](std::pair <int32_t, POSERECT<Point2f>> const &pair) { return pair.second; });
            // Checking whether a pixel belongs to the current and to another polygons            
            for (vector <POSERECT <Point2f>>::iterator iteratorPartPolygons = partPolygons.begin(); iteratorPartPolygons != partPolygons.end(); ++iteratorPartPolygons)
            {
              if ((bContainsPoint = iteratorPartPolygons->containsPoint(Point2f((float)i, (float)j)) > 0) == true)
              {
                break;; // was found polygon, which contain current pixel
              }
            }
          }
          catch (...)
          {
            stringstream ss;
            ss << "There is no such polygon for body part " << partNumber;
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
          try
          {
            vector <float> partDepths;
            multimap <int32_t, float>::iterator lower = polyDepth.lower_bound(partNumber), upper = polyDepth.upper_bound(partNumber);
            transform(lower, upper, back_inserter(partDepths), [](std::pair <int32_t, bool> const &pair) { return pair.second; }); // copy "polyDepth" to "PartDepth"
            // Checkig polygons overlapping
            for (vector <float>::iterator iteratorPartDepths = partDepths.begin(); iteratorPartDepths != partDepths.end(); ++iteratorPartDepths)
            {
              if (bContainsPoint && partHit == -1)
              {
                partHit = partNumber; // store the number of the first found polygon
                depth = *iteratorPartDepths;
              }
              else if (bContainsPoint && *iteratorPartDepths < depth) // How, for float tempDepthSign?/////////////////
              {
                partHit = partNumber;
                depth = *iteratorPartDepths;
              }
            }
          }
          catch (...)
          {
            stringstream ss;
            ss << "There is no such polyDepth parameter for body part " << partNumber;
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
        if (partHit != -1) // if was found polygon, that contains this pixel
        {
          if (!blackPixel) // if pixel color isn't black
          {
            try
            {
              partPixelColours.at(partHit).push_back(Point3i(red, green, blue)); // add colour of this pixel to part[partHit] colours
            }
            catch (...)
            {
              stringstream ss;
              ss << "There is no partPixelColours for body part " << partHit;
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
            // For all bodyparts
            for (tree <BodyPart>::iterator p = partTree.begin(); p != partTree.end(); ++p)
            {
              if ((int32_t)p->getPartID() != partHit) // if current poligon wasn't found first in the previous enumeration???
              {
                try
                {
                  bgPixelColours.at(p->getPartID()).push_back(Point3i(red, green, blue)); // add colour of this pixel to part[partHit] background colours
                }
                catch (...)
                {
                  stringstream ss;
                  ss << "There is no such bgPixelColours for body part " << p->getPartID();
                  if (debugLevelParam >= 1)
                    cerr << ERROR_HEADER << ss.str() << endl;
                  throw logic_error(ss.str());
                }
              }
            }
          }
          else
          {
            try
            {
              blankPixels.at(partHit)++; // otherwise take stock this pixel to blank pixel counter
            }
            catch (...)
            {
              stringstream ss;
              ss << "There is no such blankPixels for body part " << partHit;
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
          }
        }
        else // if  not found polygon, that contains this pixel 
        { // For all bodyparts
          for (tree <BodyPart>::iterator p = partTree.begin(); p != partTree.end(); ++p)
          {
            try
            {
              bgPixelColours.at(p->getPartID()).push_back(Point3i(red, green, blue));
            }
            catch (...)
            {
              stringstream ss;
              ss << "There is no such bgPixelColours for body part " << p->getPartID();
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
          }
        }
      }
    }

    // Create model for each bodypart
    for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
    {
      int32_t partNumber = iteratorBodyPart->getPartID();
      if (partModels.find(partNumber) == partModels.end())
      {
        PartModel model(nBins);
        partModels.insert(pair <int32_t, PartModel>(partNumber, model)); //add a new model to end of models list
      }
      try
      {
        PartModel partModel = partModels.at(partNumber);
        vector <Point3i> partPixelColoursVector; // temporary variable
        vector <Point3i> bgPixelColoursVector; // temporary variable
        int blankPixelsCount;
        try
        {
          partPixelColoursVector = partPixelColours.at(partNumber); // copy part color set for current bodypart
        }
        catch (...)
        {
          stringstream ss;
          ss << "There is no such partPixelColours for body part " << partNumber;
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
        try
        {
          blankPixelsCount = blankPixels.at(partNumber);  // copy blanck pixel count for current bodypart
        }
        catch (...)
        {
          stringstream ss;
          ss << "There is no such blankPixels for body part " << partNumber;
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
        try
        {
          bgPixelColoursVector = bgPixelColours.at(partNumber); // copy background color set for current bodypart
        }
        catch (...)
        {
          stringstream ss;
          ss << "There is no such bgPixelColours for body part " << partNumber;
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
        addPartHistogramm(partModel, partPixelColoursVector, blankPixelsCount); // building histogram for current bodypart colours
        addBackgroundHistogramm(partModel, bgPixelColoursVector); // building histograms for current bodypart background colours
        partModels.at(partNumber) = partModel; // copy result to part models set
        if (debugLevelParam >= 2)
          cerr << "Found part model: " << partNumber << endl;
      }
      catch (...)
      {
        stringstream ss;
        ss << "Could not find part model " << partNumber;
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }

    }
    delete workFrame;
    maskMat.release();
    imgMat.release();
  }
}

// Returns a labels vector of possible body parts position
vector <vector <LimbLabel> > ColorHistDetector::detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
{
  float searchDistCoeff = 0.5;
  const string sSearchDistCoeff = "searchDistCoeff";

  float minTheta = 90; // border for search
  const string sMinTheta = "minTheta";

  float maxTheta = 100; // border for search
  const string sMaxTheta = "maxTheta";

  float stepTheta = 10; // angular step of search
  const string sStepTheta = "stepTheta";

  uint32_t uniqueLocationCandidates = 4; // limiting the choice of the solutions number for each bodypart
  const string sUniqueLocationCandidates = "uniqueLocationCandidates";

  float scaleParam = 1; // scaling coefficient
  const string sScaleParam = "scaleParam";

  float searchDistCoeffMult = 1.25;
  const string sSearchDistCoeffMult = "searchDistCoeffMult";

  float useCSdet = 1.0f;
  const string sUseCSdet = "useCSdet";

#ifdef DEBUG
  uint8_t debugLevel = 5;
#else
  uint8_t debugLevel = 1;
#endif // DEBUG
  string sDebugLevel = "debugLevel";

  float rotationThreshold = 0.025f;
  const string sRotationThreshold = "rotationThreshold";

  float isWeakTreshhold = 0.1f;
  const string sIsWeakTreshhold = "isWeakTreshhold";

  float searchStepCoeff = 0.2f;
  const string sSearchStepCoeff = "searchStepCoeff";

  // first we need to check all used params
  params.emplace(sSearchDistCoeff, searchDistCoeff);
  params.emplace(sMinTheta, minTheta);
  params.emplace(sMaxTheta, maxTheta);
  params.emplace(sStepTheta, stepTheta);
  params.emplace(sUniqueLocationCandidates, uniqueLocationCandidates);
  params.emplace(sScaleParam, scaleParam);
  params.emplace(sSearchDistCoeffMult, searchDistCoeffMult);
  params.emplace(sUseCSdet, useCSdet);
  params.emplace(sDebugLevel, debugLevel);
  params.emplace(sRotationThreshold, rotationThreshold);
  params.emplace(sIsWeakTreshhold, isWeakTreshhold);
  params.emplace(sSearchStepCoeff, searchStepCoeff);

  //now set actual param values
  useCSdet = params.at(sUseCSdet);
  searchDistCoeff = params.at(sSearchDistCoeff);

  minTheta = params.at(sMinTheta);
  maxTheta = params.at(sMaxTheta);
  stepTheta = params.at(sStepTheta);
  uniqueLocationCandidates = params.at(sUniqueLocationCandidates);
  scaleParam = params.at(sScaleParam);
  searchDistCoeffMult = params.at(sSearchDistCoeffMult);
  useCSdet = params.at(sUseCSdet);
  debugLevel = params.at(sDebugLevel);
  rotationThreshold = params.at(sRotationThreshold);
  isWeakTreshhold = params.at(sIsWeakTreshhold);
  searchStepCoeff = params.at(sSearchStepCoeff);
  debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

  int originalSize = frame->getFrameSize().height;

  Frame *workFrame = 0;
  if (frame->getFrametype() == KEYFRAME)
    workFrame = new Keyframe();
  else if (frame->getFrametype() == LOCKFRAME)
    workFrame = new Lockframe();
  else if (frame->getFrametype() == INTERPOLATIONFRAME)
    workFrame = new Interpolation();

  workFrame = frame->clone(workFrame);

  float resizeFactor = workFrame->Resize(maxFrameHeight);

  vector <vector <LimbLabel> > t;
  Skeleton skeleton = workFrame->getSkeleton(); // copy skeleton from the frame
  tree <BodyPart> partTree = skeleton.getPartTree(); // copy tree of bodypart from the skeleton

  map <int32_t, Mat> pixelDistributions = buildPixelDistributions(workFrame); // matrix contains the probability that the particular pixel belongs to current bodypart
  map <int32_t, Mat> pixelLabels = buildPixelLabels(workFrame, pixelDistributions); // matrix contains relative estimations that the particular pixel belongs to current bodypart
  Mat maskMat = workFrame->getMask(); // copy mask from the frame

  stringstream detectorName;
  detectorName << getID();

  // For all body parts
  for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  { //Temporary variables
    vector <LimbLabel> labels;
    vector <LimbLabel> sortedLabels;
    Point2f j0, j1;

    try
    {
      j0 = skeleton.getBodyJoint(iteratorBodyPart->getParentJoint())->getImageLocation(); // copy current bodypart parent joint
      j1 = skeleton.getBodyJoint(iteratorBodyPart->getChildJoint())->getImageLocation(); // copy current bodypart child joint
    }
    catch (...)
    {
      stringstream ss;
      ss << "Can't get joints";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }

    float boneLength = getBoneLength(j0, j1); // distance between nodes
    float boxWidth = getBoneWidth(boneLength, *iteratorBodyPart); // current body part polygon width
    Point2f direction = j1 - j0; // direction of bodypart vector
    float theta = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));  // bodypart tilt angle 
    float minDist = boxWidth * params.at(sSearchStepCoeff); // linear step of searching
    if (minDist < 2) minDist = 2; // the minimal linear step
    float searchDistance = iteratorBodyPart->getSearchRadius();
    try
    {
      if (searchDistance <= 0)
        searchDistance = boneLength * params.at(sSearchDistCoeff); // the limiting of search area
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sSearchDistCoeff << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    Point2f suggestStart = 0.5 * j1 + 0.5 * j0; // reference point - the bodypart center
    // Scan the area around the reference point
    for (float x = suggestStart.x - searchDistance * 0.5f; x < suggestStart.x + searchDistance * 0.5f; x += minDist)
    {
      for (float y = suggestStart.y - searchDistance * 0.5f; y < suggestStart.y + searchDistance * 0.5f; y += minDist)
      {
        if (x < maskMat.cols && y < maskMat.rows)
        {
          uint8_t mintensity = 0;
          try
          {
            mintensity = maskMat.at<uint8_t>((int)y, (int)x); // copy mask at current pixel
          }
          catch (...)
          {
            stringstream ss;
            ss << "Can't get value in maskMat at " << "[" << (int)y << "][" << (int)x << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
          bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
          if (!blackPixel)
          { // Scan the possible rotation zone
            float deltaTheta = abs(iteratorBodyPart->getRotationSearchRange());// + abs(rotationThreshold);
            float maxLocalTheta = iteratorBodyPart->getRotationSearchRange() == 0 ? maxTheta : deltaTheta;
            float minLocalTheta = iteratorBodyPart->getRotationSearchRange() == 0 ? minTheta : deltaTheta;
            for (float rot = theta - minLocalTheta; rot < theta + maxLocalTheta; rot += stepTheta)
            {
              // Create a new label vector and build it label
              Point2f p0 = Point2f(0, 0); // the point of unit vector
              Point2f p1 = Point2f(1.0, 0); // the point of unit vector
              p1 *= boneLength; // change the vector length 
              p1 = PoseHelper::rotatePoint2D(p1, p0, rot); // rotate the vector
              Point2f mid = 0.5 * p1; // center of the vector
              p1 = p1 + Point2f(x, y) - mid; // shift the vector to current point
              p0 = Point2f(x, y) - mid; // shift the vector to current point


              LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, workFrame, pixelDistributions, pixelLabels, p0, p1, useCSdet); // build  the vector label

              if (generatedLabel.getPolygon().size() != 4 || generatedLabel.getScores().size() < 1 || generatedLabel.getScores().size() > 3)
              {
                cerr << "here it is..." << endl;
              }

              sortedLabels.push_back(generatedLabel); // add label to current bodypart labels
            }
          }
        }
      }
    }
    if (sortedLabels.size() == 0) // if labels for current body part is not builded
    {
      for (float rot = theta - minTheta; (rot < theta + maxTheta || (rot == theta - minTheta && rot >= theta + maxTheta)); rot += stepTheta)
      {
        Point2f p0 = Point2f(0, 0); // the point of unit vector
        Point2f p1 = Point2f(1.0, 0); // the point of unit vector
        p1 *= boneLength; // change the vector length 
        p1 = PoseHelper::rotatePoint2D(p1, p0, rot); // rotate the vector
        Point2f mid = 0.5 * p1; // center of the vector
        p1 = p1 + Point2f(suggestStart.x, suggestStart.y) - mid; // shift the vector to reference point
        p0 = Point2f(suggestStart.x, suggestStart.y) - mid; // shift the vector to reference point
        LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, workFrame, pixelDistributions, pixelLabels, p0, p1, useCSdet);
        sortedLabels.push_back(generatedLabel); // add label to current bodypart labels
      }
    }
    float uniqueLocationCandidates = 0;
    try
    {
      uniqueLocationCandidates = params.at(sUniqueLocationCandidates); // copy the value from input parameters
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sUniqueLocationCandidates << "' param";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    if (sortedLabels.size() > 0) // if labels vector is not empty
    {
      sort(sortedLabels.begin(), sortedLabels.end()); // sort labels by "SumScore" ?
      Mat locations(workFrame->getFrameSize().height, workFrame->getFrameSize().width, DataType<uint32_t>::type); // create the temporary matrix
      for (int32_t i = 0; i < workFrame->getFrameSize().width; i++)
      {
        for (int32_t j = 0; j < workFrame->getFrameSize().height; j++)
        {
          try
          {
            locations.at<uint32_t>(j, i) = 0; // init elements of the "location" matrix
          }
          catch (...)
          {
            stringstream ss;
            ss << "There is no value of locations at " << "[" << j << "][" << i << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
      }
      // For all "sortedLabels"
      for (uint32_t i = 0; i < sortedLabels.size(); i++)
      {
        if (sortedLabels[i].getPolygon().size() != 4 || sortedLabels[i].getScores().size() < 1 || sortedLabels[i].getScores().size() > 3)
        {
          cerr << "here it is..." << endl;
        }

        uint32_t x = (uint32_t)sortedLabels.at(i).getCenter().x; // copy center coordinates of current label
        uint32_t y = (uint32_t)sortedLabels.at(i).getCenter().y; // copy center coordinates of current label
        try
        {
          if (locations.at<uint32_t>(y, x) < uniqueLocationCandidates) // current point is occupied by less then "uniqueLocationCandidates" of labels with a greater score
          {
            try
            {
              labels.push_back(sortedLabels.at(i)); // add the current label in the resulting set of labels
            }
            catch (...)
            {
              stringstream ss;
              ss << "Maybe there is no value of sortedLabels at " << "[" << i << "]";
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
            locations.at<uint32_t>(y, x) += 1; // increase the counter of labels number at given point
          }
        }
        catch (...)
        {
          stringstream ss;
          ss << "Maybe there is no value of locations at " << "[" << y << "][" << x << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
        if (sortedLabels[i].getPolygon().size() != 4 || sortedLabels[i].getScores().size() < 1 || sortedLabels[i].getScores().size() > 3)
        {
          cerr << "here it is..." << endl;
        }
      }
      locations.release();
      for (uint32_t i = 0; i < labels.size(); i++)
      {
        if (labels[i].getPolygon().size() != 4 || labels[i].getScores().size() < 1 || labels[i].getScores().size() > 3)
        {
          cerr << "here it is..." << endl;
        }
      }
    }
    PoseHelper::RecalculateScoreIsWeak(labels, detectorName.str(), isWeakTreshhold);
    if (labels.size() > 0)
      t.push_back(labels); // add current point labels

    for (uint32_t i = 0; i < sortedLabels.size(); i++)
    {
      if (sortedLabels[i].getPolygon().size() != 4 || sortedLabels[i].getScores().size() < 1 || sortedLabels[i].getScores().size() > 3)
      {
        cerr << "here it is..." << endl;
      }
    }
  }
  map <int32_t, Mat>::iterator i;
  for (i = pixelDistributions.begin(); i != pixelDistributions.end(); ++i)
  {
    i->second.release();
  }
  maskMat.release();

  delete workFrame;

  for (auto i = 0; i < t.size(); ++i)
  {
    for (auto j = 0; j < t.at(i).size(); ++j)
    {
      LimbLabel label = t.at(i).at(j);
      label.Resize(pow(resizeFactor, -1));
      t[i][j] = label;
    }
  }

  return merge(limbLabels, t);
}

// Return nBins
uint8_t ColorHistDetector::getNBins(void) const
{
  return nBins;
}

// Returns relative frequency of the RGB-color reiteration in "PartModel" 
float ColorHistDetector::computePixelBelongingLikelihood(const PartModel &partModel, uint8_t r, uint8_t g, uint8_t b)
{ // Scaling of colorspace, finding the colors interval, which now gets this color
  uint8_t factor = static_cast<uint8_t> (ceil(pow(2, 8) / partModel.nBins));
  try
  {
    return partModel.partHistogramm.at(r / factor).at(g / factor).at(b / factor); // relative frequency of current color reiteration 
  }
  catch (...)
  {
    stringstream ss;
    ss << "Couldn't find partHistogramm " << "[" << (int)r / factor << "][" << (int)g / factor << "][" << (int)b / factor << "]";
    if (debugLevelParam >= 1)
      cerr << ERROR_HEADER << ss.str() << endl;
    throw logic_error(ss.str());
  }
}

// Build into the "partModel" a histogram of the color set "partColors"
void ColorHistDetector::setPartHistogramm(PartModel &partModel, const vector <Point3i> &partColors)
{
  // do not add sample if the number of pixels is zero
  if (partColors.size() == 0)
    return;
  uint8_t factor = static_cast<uint8_t> (ceil(pow(2, 8) / partModel.nBins)); // colorspace scaling coefficient
  partModel.sizeFG = static_cast <uint32_t> (partColors.size());
  partModel.fgNumSamples = 1;
  partModel.fgSampleSizes.clear();
  partModel.fgSampleSizes.push_back(static_cast <uint32_t> (partColors.size()));

  // clear histogram first
  for (uint8_t r = 0; r < partModel.nBins; r++)
  {
    for (uint8_t g = 0; g < partModel.nBins; g++)
    {
      for (uint8_t b = 0; b < partModel.nBins; b++)
      {
        try
        {
          partModel.partHistogramm.at(r).at(g).at(b) = 0.0;
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't find partHistogramm " << "[" << r << "][" << g << "][" << b << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
      }
    }
  }
  // Scaling of colorspace, reducing the capacity and number of colour intervals that are used to construct the histogram
  for (uint32_t i = 0; i < partColors.size(); i++)
  {
    uint8_t r, g, b;
    try
    {
      r = static_cast<uint8_t> (partColors.at(i).x / factor);
      g = static_cast<uint8_t> (partColors.at(i).y / factor);
      b = static_cast<uint8_t> (partColors.at(i).z / factor);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't get partColors with index " << i;
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    try
    {
      partModel.partHistogramm.at(r).at(g).at(b)++; // increment the frequency of interval, that this color have hit
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't find partHistogramm " << "[" << r << "][" << g << "][" << b << "]";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
  }
  for (uint8_t r = 0; r < partModel.nBins; r++)
  {
    for (uint8_t g = 0; g < partModel.nBins; g++)
    {
      for (uint8_t b = 0; b < partModel.nBins; b++)
      {
        // normalise the histograms
        try
        {
          partModel.partHistogramm.at(r).at(g).at(b) /= partModel.sizeFG;
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't find partHistogramm " << "[" << r << "][" << g << "][" << b << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
      }
    }
  }
}

// Take stock of the additional set of colors in the histogram
void ColorHistDetector::addPartHistogramm(PartModel &partModel, const vector <Point3i> &partColors, uint32_t nBlankPixels)
{
  if (partColors.size() == 0) //do not add sample if the number of pixels is zero
    return;
  //un-normalise
  for (uint8_t r = 0; r < partModel.nBins; r++)
  {
    for (uint8_t g = 0; g < partModel.nBins; g++)
    {
      for (uint8_t b = 0; b < partModel.nBins; b++)
      {
        try
        {
          partModel.partHistogramm.at(r).at(g).at(b) *= partModel.sizeFG; // converting the colors relative frequency into the pixels number
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't find partHistogramm " << "[" << r << "][" << g << "][" << b << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
      }
    }
  }

  int factor = (int)ceil(pow(2, 8) / partModel.nBins); // colorspace scaling coefficient
  partModel.sizeFG += static_cast <uint32_t> (partColors.size());
  partModel.fgNumSamples++;
  partModel.fgSampleSizes.push_back(static_cast <uint32_t> (partColors.size()));

  // Scaling of colorspace, reducing the capacity and number of colour intervals
  // Adjustment of the histogram
  for (uint32_t i = 0; i < partColors.size(); i++)
  {
    Point3i color;
    try
    {
      color = partColors.at(i);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't find partColor " << "[" << i << "]";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    uint8_t r = static_cast<uint8_t> (color.x / factor);
    uint8_t g = static_cast<uint8_t> (color.y / factor);
    uint8_t b = static_cast<uint8_t> (color.z / factor);
    try
    {
      partModel.partHistogramm.at(r).at(g).at(b)++; // increment the frequency of interval, that this color have hit
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't find partHistogramm " << "[" << r << "][" << g << "][" << b << "]";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
  }

  //renormalise
  for (uint8_t r = 0; r < partModel.nBins; r++)
  {
    for (uint8_t g = 0; g < partModel.nBins; g++)
    {
      for (uint8_t b = 0; b < partModel.nBins; b++)
      {
        //normalise the histograms
        try
        {
          partModel.partHistogramm.at(r).at(g).at(b) /= partModel.sizeFG;
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't find partHistogramm " << "[" << r << "][" << g << "][" << b << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
      }
    }
  }

  partModel.fgBlankSizes.push_back(nBlankPixels); // add the number of blank pixels for this model
}

// Totalization the number of used samples
float ColorHistDetector::getAvgSampleSizeFg(const PartModel &partModel)
{
  float sum = 0;
  for (uint32_t i = 0; i < partModel.fgSampleSizes.size(); i++)
  {
    try
    {
      sum += partModel.fgSampleSizes.at(i);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't find fgSampleSizes " << "[" << i << "]";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
  }
  sum /= partModel.fgNumSamples;
  return sum;
}

// Averaging the number of samples, that united from two sets
float ColorHistDetector::getAvgSampleSizeFgBetween(const PartModel &partModel, uint32_t s1, uint32_t s2)
{
  if (s1 >= partModel.fgSampleSizes.size() || s2 >= partModel.fgSampleSizes.size())
    return 0;
  try
  {
    return (partModel.fgSampleSizes.at(s1) + partModel.fgSampleSizes.at(s2)) / 2.0f;
  }
  catch (...)
  {
    stringstream ss;
    ss << "Couldn't find fgSampleSizes " << "[" << s1 << "] or fgSampleSizes [" << s2 << "]";
    if (debugLevelParam >= 1)
      cerr << ERROR_HEADER << ss.str() << endl;
    throw logic_error(ss.str());
  }
}

//TODO (Vitaliy Koshura): Need unit test
// Euclidean distance between part histograms
float ColorHistDetector::matchPartHistogramsED(const PartModel &partModelPrev, const PartModel &partModel)
{
  float distance = 0;
  for (uint8_t r = 0; r < partModel.nBins; r++)
  {
    for (uint8_t g = 0; g < partModel.nBins; g++)
    {
      for (uint8_t b = 0; b < partModel.nBins; b++)
      {
        // accumulation of the Euclidean distances between the points
        try
        {
          distance += pow(partModel.partHistogramm.at(r).at(g).at(b) - partModelPrev.partHistogramm.at(r).at(g).at(b), 2);
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't find partHistogramm " << "[" << r << "][" << g << "][" << b << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
      }
    }
  }
  return sqrt(distance);
}

// Background histogram
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
        try
        {
          partModel.bgHistogramm.at(r).at(g).at(b) *= partModel.sizeBG;
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't find bgHistogramm " << "[" << r << "][" << g << "][" << b << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
      }
    }
  }
  uint32_t factor = (uint32_t)ceil(pow(2, 8) / partModel.nBins); // colorspace scaling coefficient
  partModel.sizeBG += static_cast <uint32_t> (bgColors.size());
  partModel.bgNumSamples++;
  partModel.bgSampleSizes.push_back(static_cast <uint32_t> (bgColors.size()));
  for (uint32_t i = 0; i < bgColors.size(); i++)
  {
    Point3i color;
    try
    {
      color = bgColors.at(i);
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't find bgColors " << "[" << i << "]";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    try
    {
      partModel.bgHistogramm.at(color.x / factor).at(color.y / factor).at(color.z / factor)++; // increment the frequency of interval, that this color have hit
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't find bgHistogramm " << "[" << color.x / factor << "][" << color.y / factor << "][" << color.z / factor << "]";
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
  }
  // renormalise
  for (uint8_t r = 0; r < partModel.nBins; r++)
  {
    for (uint8_t g = 0; g < partModel.nBins; g++)
    {
      for (uint8_t b = 0; b < partModel.nBins; b++)
      {
        try
        {
          partModel.bgHistogramm.at(r).at(g).at(b) /= (float)partModel.sizeBG;
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't find bgHistogramm " << "[" << r << "][" << g << "][" << b << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
      }
    }
  }
}

// Returns a matrix, that contains relative frequency of the pixels colors reiteration 
map <int32_t, Mat> ColorHistDetector::buildPixelDistributions(Frame *frame)
{
  Skeleton skeleton = frame->getSkeleton(); // copy skeleton from the frame
  tree <BodyPart> partTree = skeleton.getPartTree(); // copy part tree from the skeleton
  Mat imgMat = frame->getImage(); // copy image from the frame
  Mat maskMat = frame->getMask(); // copy mask from the frame
  uint32_t width = imgMat.cols;
  uint32_t height = imgMat.rows;
  uint32_t mwidth = maskMat.cols;
  uint32_t mheight = maskMat.rows;
  map <int32_t, Mat> pixelDistributions;
  if (width != mwidth || height != mheight) // error if mask and image sizes don't match
  {
    stringstream ss;
    ss << "Mask size not equal image size";
    if (debugLevelParam >= 1)
      cerr << ERROR_HEADER << ss.str() << endl;
    throw logic_error(ss.str());
  }
  // For all bodyparts
  for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    Mat t = Mat(height, width, DataType <float>::type); // create empty matrix
    int partID = iteratorBodyPart->getPartID();
    try
    {
      PartModel partModel = partModels.at(partID); // copy part model of current bodybart
      // For all pixels
      for (uint32_t x = 0; x < width; x++)
      {
        for (uint32_t y = 0; y < height; y++)
        {
          Vec3b intensity = imgMat.at<Vec3b>(y, x);
          // Copy components of the current pixel color
          uint8_t blue = intensity.val[0];
          uint8_t green = intensity.val[1];
          uint8_t red = intensity.val[2];
          uint8_t mintensity = maskMat.at<uint8_t>(y, x); // copy mask of the current pixel
          bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold

          t.at<float>(y, x) = blackPixel ? 0 : computePixelBelongingLikelihood(partModel, red, green, blue); // relative frequency of the current pixel color reiteration 
        }
      }
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe couldn't find partModel " << partID;
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    pixelDistributions.insert(pair <int32_t, Mat>(partID, t)); // add the current bodypart matrix to the set 
    t.release();
  }
  imgMat.release();
  maskMat.release();
  return pixelDistributions;
}


map <int32_t, Mat> ColorHistDetector::buildPixelLabels(Frame *frame, map <int32_t, Mat> pixelDistributions)
{
  Mat maskMat = frame->getMask(); // copy mask from the frame
  uint32_t width = maskMat.cols;
  uint32_t height = maskMat.rows;
  Skeleton skeleton = frame->getSkeleton(); // copy skeleton from the frame
  tree <BodyPart> partTree = skeleton.getPartTree(); // copy part tree from the skeleton
  map <int32_t, Mat> pixelLabels;
  // For all body parts
  for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    Mat t = Mat(height, width, DataType <float>::type); // create empty matrix
    Mat tt;
    try
    { // Matrix, that contains relative frequency of the pixels colors reiteration for current body part
      tt = pixelDistributions.at(iteratorBodyPart->getPartID());
    }
    catch (...)
    {
      stringstream ss;
      ss << "Couldn't find distributions for body part " << iteratorBodyPart->getPartID();
      if (debugLevelParam >= 1)
        cerr << ERROR_HEADER << ss.str() << endl;
      throw logic_error(ss.str());
    }
    // For all pixels
    for (uint32_t x = 0; x < width; x++)
    {
      for (uint32_t y = 0; y < height; y++)
      {
        uint8_t mintensity = maskMat.at<uint8_t>(y, x); //copy the current pixel mask value
        bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
        if (!blackPixel)
        {
          float top = 0;
          float sum = 0;
          // For all body parts
          for (tree <BodyPart>::iterator i = partTree.begin(); i != partTree.end(); ++i)
          {
            Mat temp;
            try
            {
              temp = pixelDistributions.at(i->getPartID()); // matrix of the pixels colors frequency for current body part
            }
            catch (...)
            {
              stringstream ss;
              ss << "Couldn't find pixel distributions for body part " << i->getPartID();
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
            try
            {
              if (temp.at<float>(y, x) > top) // search max value of the current bodypart pixel color frequency
                top = temp.at<float>(y, x);
              sum += temp.at<float>(y, x);
              temp.release();
            }
            catch (...)
            {
              stringstream ss;
              ss << "Couldn't find value of temp " << "[" << y << "][" << x << "]";
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
          }
          try
          {
            t.at<float>(y, x) = (top == 0) ? 0 : tt.at<float>(y, x) / (float)top;
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't find t " << "[" << y << "][" << x << "] or tt [" << y << "][" << x << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
        else
        {
          try
          {
            t.at<float>(y, x) = 0;
          }
          catch (...)
          {
            stringstream ss;
            ss << "Couldn't find value of t " << "[" << y << "][" << x << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
      }
    }
    pixelLabels.insert(pair<int32_t, Mat>(iteratorBodyPart->getPartID(), t)); // insert the resulting matrix into the set "pixelLabels" 
    t.release();
  }
  maskMat.release();
  map <int32_t, Mat>::iterator i;
  for (i = pixelDistributions.begin(); i != pixelDistributions.end(); ++i)
  {
    i->second.release(); // delete pixelDistributions 
  }
  return pixelLabels;
}

float ColorHistDetector::compare(void)
{
  if (comparer_bodyPart == 0 || comparer_frame == 0 || comparer_pixelDistributions == 0 || comparer_pixelLabels == 0 || comparer_j0 == 0 || comparer_j1 == 0)
  {
    stringstream ss;
    ss << "Compare parameters are invalid: " << (comparer_bodyPart == 0 ? "comparer_bodyPart == 0 " : "") << (comparer_frame == 0 ? "comparer_frame == 0 " : "") << (comparer_pixelDistributions == 0 ? "comparer_pixelDistributions == 0" : "") << (comparer_pixelLabels == 0 ? "comparer_pixelLabels == 0" : "") << (comparer_j0 == 0 ? "comparer_j0 == 0" : "") << (comparer_j1 == 0 ? "comparer_j1 == 0" : "") << endl;
    if (debugLevelParam >= 1)
      cerr << ERROR_HEADER << ss.str() << endl;
    throw logic_error(ss.str());
  }
  try
  {
    return compare(*comparer_bodyPart, *comparer_frame, *comparer_pixelDistributions, *comparer_pixelLabels, *comparer_j0, *comparer_j1);
  }
  catch (logic_error ex)
  {
    if (debugLevelParam >= 1)
      cerr << ERROR_HEADER << "Dirty Label: " << ex.what() << endl;
    return -1.0f;
  }
}

float ColorHistDetector::compare(BodyPart bodyPart, Frame *frame, map <int32_t, Mat> pixelDistributions, map <int32_t, Mat> pixelLabels, Point2f j0, Point2f j1)
{
  Mat maskMat = frame->getMask(); // copy mask from the frame 
  Mat imgMat = frame->getImage(); // copy image from the frame
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5; // segment center
  float boneLength = getBoneLength(j0, j1); // distance between joints
  POSERECT <Point2f> rect = getBodyPartRect(bodyPart, j0, j1); // expected bodypart location area?
  uint32_t totalPixels = 0;
  uint32_t pixelsInMask = 0;
  float totalPixelLabelScore = 0;
  float pixDistAvg = 0;
  float pixDistNum = 0;
  PartModel model;
  try
  {
    model = partModels.at(bodyPart.getPartID()); // copy part model for the "bodyPart"
  }
  catch (...)
  {
    maskMat.release();
    imgMat.release();
    stringstream ss;
    ss << "Couldn't get partModel of bodyPart " << bodyPart.getPartID();
    if (debugLevelParam >= 1)
      cerr << ERROR_HEADER << ss.str() << endl;
    throw logic_error(ss.str());
  }
  if (getAvgSampleSizeFg(model) == 0) // error if samples count is zero
  {
    maskMat.release();
    imgMat.release();
    stringstream ss;
    ss << "Couldn't get avgSampleSizeFg";
    if (debugLevelParam >= 2)
      cerr << ERROR_HEADER << ss.str() << endl;
    throw logic_error(ss.str());
  }
  float xmax, ymax, xmin, ymin;
  rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax); // highlight the extreme points of the body part rect
  Mat bodyPartPixelDistribution;
  try
  {
    bodyPartPixelDistribution = pixelDistributions.at(bodyPart.getPartID());
  }
  catch (...)
  {
    maskMat.release();
    imgMat.release();
    stringstream ss;
    ss << "Can't get pixesDistribution [" << bodyPart.getPartID() << "]";
    if (debugLevelParam >= 2)
      cerr << ERROR_HEADER << ss.str() << endl;
    throw logic_error(ss.str());
  }
  Mat bodyPartLixelLabels;
  try
  {
    bodyPartLixelLabels = pixelLabels.at(bodyPart.getPartID());
  }
  catch (...)
  {
    maskMat.release();
    imgMat.release();
    bodyPartPixelDistribution.release();
    stringstream ss;
    ss << "Can't get pixesLabels [" << bodyPart.getPartID() << "]";
    if (debugLevelParam >= 2)
      cerr << ERROR_HEADER << ss.str() << endl;
    throw logic_error(ss.str());
  }
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
              bodyPartPixelDistribution.release();
              bodyPartLixelLabels.release();
              stringstream ss;
              ss << "Can't get maskMat [" << j << "][" << i << "]";
              if (debugLevelParam >= 2)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
            bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
            if (!blackPixel)
            {
              try
              {
                pixDistAvg += bodyPartPixelDistribution.at<float>(j, i); // Accumulation the "distributions" of contained pixels
              }
              catch (...)
              {
                maskMat.release();
                imgMat.release();
                bodyPartPixelDistribution.release();
                bodyPartLixelLabels.release();
                stringstream ss;
                ss << "Can't get pixesDistribution [" << bodyPart.getPartID() << "][" << j << "][" << i << "]";
                if (debugLevelParam >= 2)
                  cerr << ERROR_HEADER << ss.str() << endl;
                throw logic_error(ss.str());
              }
              pixDistNum++; // counting of the all scanned pixels
              try
              {
                if (bodyPartLixelLabels.at<float>(j, i))
                {
                  totalPixelLabelScore += bodyPartLixelLabels.at<float>(j, i); // Accumulation of the pixel labels
                }
              }
              catch (...)
              {
                maskMat.release();
                imgMat.release();
                bodyPartPixelDistribution.release();
                bodyPartLixelLabels.release();
                stringstream ss;
                ss << "Can't get pixesLabels [" << bodyPart.getPartID() << "][" << j << "][" << i << "]";
                if (debugLevelParam >= 2)
                  cerr << ERROR_HEADER << ss.str() << endl;
                throw logic_error(ss.str());
              }
              pixelsInMask++; // counting pixels within the mask
            }
          }
        }
      }
    }
  }
  maskMat.release();
  imgMat.release();
  bodyPartPixelDistribution.release();
  bodyPartLixelLabels.release();
  float supportScore = 0;
  float inMaskSupportScore = 0;
  pixDistAvg /= (float)pixDistNum;  // average "distributions"
  float inMaskSuppWeight = 0.5;
  if (totalPixelLabelScore > 0 && totalPixels > 10)
  {
    supportScore = (float)totalPixelLabelScore / (float)totalPixels;
    inMaskSupportScore = (float)totalPixelLabelScore / (float)pixelsInMask;
    float score = 1.0f - ((1.0f - inMaskSuppWeight) * supportScore + inMaskSuppWeight * inMaskSupportScore);
    return score;
  }
  stringstream ss;
  ss << "Dirty label!";
  if (debugLevelParam >= 2)
    cerr << ERROR_HEADER << ss.str() << endl;
  throw logic_error(ss.str());
}

LimbLabel ColorHistDetector::generateLabel(BodyPart bodyPart, Frame *frame, map <int32_t, Mat> pixelDistributions, map <int32_t, Mat> pixelLabels, Point2f j0, Point2f j1, float _useCSdet)
{
  stringstream detectorName;
  detectorName << getID();

  comparer_bodyPart = &bodyPart;
  comparer_frame = &frame;
  comparer_pixelDistributions = &pixelDistributions;
  comparer_pixelLabels = &pixelLabels;
  comparer_j0 = &j0;
  comparer_j1 = &j1;

  LimbLabel label = Detector::generateLabel(bodyPart, j0, j1, detectorName.str(), _useCSdet);

  comparer_bodyPart = 0;
  comparer_frame = 0;
  comparer_pixelDistributions = 0;
  comparer_pixelLabels = 0;
  comparer_j0 = 0;
  comparer_j1 = 0;

  return label;
}

//Used only as prevent a warning for "const uint8_t nBins";
vector <Frame*> ColorHistDetector::getFrames() const
{
  return frames;
}

//Used only as prevent a warning for "const uint8_t nBins";
ColorHistDetector &ColorHistDetector::operator=(const ColorHistDetector &c)
{
  this->frames = c.getFrames();
  return *this;
}
