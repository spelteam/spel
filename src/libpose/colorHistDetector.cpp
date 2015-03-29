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

// Copy all fields of the "PartModel" structure
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

// Constructor with initialization of constant field "nBins"
ColorHistDetector::ColorHistDetector(uint8_t _nBins) : nBins(_nBins)
{
  id = 0x434844;
}

// Returns unique ID of "ColorHistDetector" object
int ColorHistDetector::getID(void)
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
  const float scaleParam = 1; // scaling coefficient
  const string sScaleParam = "scaleParam";
  const uint8_t debugLevel = 1;
  const string sDebugLevel = "debugLevel";
  // first we need to check all used params
  params.emplace(sScaleParam, scaleParam);
  params.emplace(sDebugLevel, debugLevel);

  debugLevelParam = params.at(sDebugLevel);

  if (frames.size() == 0)
    throw logic_error("No input frames"); // the sequence of frames is empty
  partModels.clear();
  // Find skeleton from first keyframe or lockframe
  Skeleton skeleton;
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
  tree <BodyPart>::iterator iteratorBodyPart;
  for (vector <Frame*>::iterator frameNum = frames.begin(); frameNum != frames.end(); ++frameNum)
  {
    if ((*frameNum)->getFrametype() != KEYFRAME && (*frameNum)->getFrametype() != LOCKFRAME)
    {
      continue; // skip unmarked frames
    }
    if (debugLevelParam >= 2)
      cerr << "Training on frame " << (*frameNum)->getID() << endl;
    // Create local variables
    map <int32_t, vector <Point3i>> partPixelColours; // the set of RGB-colours of pixel's for current body part
    map <int32_t, vector <Point3i>> bgPixelColours; // the set of RGB-colours for a pixels of background
    map <int32_t, int> blankPixels;  // pixels outside the mask
    skeleton = (*frameNum)->getSkeleton(); // copy marking from current frame
    multimap <int32_t, POSERECT <Point2f>> polygons;  // polygons for this frame
    multimap <int32_t, float> polyDepth; // used for evaluation of overlapped polygons
    partTree = skeleton.getPartTree(); // the skeleton body parts
    tree <BodyJoint> jointTree; // the skeleton joints
    tree <BodyJoint>::iterator iteratorBodyJoint;
    // Handling all bodyparts on the frames
    for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
    {
      partPixelColours.insert(pair <int32_t, vector <Point3i>>(iteratorBodyPart->getPartID(), vector <Point3i>())); // container initialization for conserve colours set for each of body parts
      bgPixelColours.insert(pair <int32_t, vector <Point3i>>(iteratorBodyPart->getPartID(), vector <Point3i>())); // container initialization for conserve background colours set for each of body parts
      blankPixels.insert(pair <int32_t, int>(iteratorBodyPart->getPartID(), 0)); // container initialization for counting blank pixels for each of body parts
      Point2f j1, j0;  // temporary adjacent joints
      Point2f c1, c2, c3, c4; // temporary polygon's vertices
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
      float boneLength = (float)sqrt(PoseHelper::distSquared(j0, j1)); // distance between nodes
      //TODO (Vitaliy Koshura): Check this!
      float boneWidth = 0;
      try
      { //currents body part polygon width 
        boneWidth = boneLength / iteratorBodyPart->getLWRatio();
      }
      catch (...)
      {
        stringstream ss;
        ss << "Maybe there is no '" << sScaleParam << "' param";
        if (debugLevelParam >=1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
      Point2f boxCenter = j0 * 0.5 + j1 * 0.5; // the bobypart center  coordinates
      // Coordinates for drawing of the polygon at the coordinate origin
      c1 = Point2f(0.f, 0.5f * boneWidth);
      c2 = Point2f(boneLength, 0.5f * boneWidth);
      c3 = Point2f(boneLength, -0.5f * boneWidth);
      c4 = Point2f(0.f, -0.5f * boneWidth);
      Point2f polyCenter = Point2f(boneLength * 0.5f, 0.f); // polygon center 
      Point2f direction = j1 - j0; // used as estimation of the vector's direction
      float rotationAngle = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI)); //bodypart tilt angle 
      // Rotate and shift the polygon to the bodypart center
      c1 = PoseHelper::rotatePoint2D(c1, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c2 = PoseHelper::rotatePoint2D(c2, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c3 = PoseHelper::rotatePoint2D(c3, polyCenter, rotationAngle) + boxCenter - polyCenter;
      c4 = PoseHelper::rotatePoint2D(c4, polyCenter, rotationAngle) + boxCenter - polyCenter;
      POSERECT <Point2f> poserect(c1, c2, c3, c4);
      polygons.insert(pair <int32_t, POSERECT <Point2f>>(iteratorBodyPart->getPartID(), poserect));
      polyDepth.insert(pair <int32_t, float>(iteratorBodyPart->getPartID(), skeleton.getBodyJoint(iteratorBodyPart->getParentJoint())->getSpaceLocation().z));
    }
    Mat maskMat = (*frameNum)->getMask(); // copy mask from the current frame
    Mat imgMat = (*frameNum)->getImage(); // copy image from the current frame
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
        for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
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
    for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
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
    maskMat.release();
    imgMat.release();
  }
}

// Returns a labels vector of possible body parts position
vector <vector <LimbLabel> > ColorHistDetector::detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
{
  const float searchDistCoeff = 0.5;
  const string sSearchDistCoeff = "searchDistCoeff";

  const float minTheta = 90; // border for search 
  const string sMinTheta = "minTheta";

  const float maxTheta = 100; // border for search 
  const string sMaxTheta = "maxTheta";

  const float stepTheta = 10; // angular step of search 
  const string sStepTheta = "stepTheta";

  const uint32_t uniqueLocationCandidates = 4; // limiting the choice of the solutions number for each bodypart 
  const string sUniqueLocationCandidates = "uniqueLocationCandidates";

  const float scaleParam = 1; // scaling coefficient
  const string sScaleParam = "scaleParam";

  const float searchDistCoeffMult = 1.25;
  const string sSearchDistCoeffMult = "searchDistCoeffMult";

  const float useCSdet = 1.0f;
  const string sUseCSdet = "useCSdet";

  const uint8_t debugLevel = 1;
  const string sDebugLevel = "debugLevel";


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

  debugLevelParam = params.at(sDebugLevel);

  vector <vector <LimbLabel> > t;
  Skeleton skeleton = frame->getSkeleton(); // copy skeleton from the frame
  tree <BodyPart> partTree = skeleton.getPartTree(); // copy tree of bodypart from the skeleton

  tree <BodyPart>::iterator iteratorBodyPart;
  map <int32_t, Mat> pixelDistributions = buildPixelDistributions(frame); // matrix contains the probability that the particular pixel belongs to current bodypart
  map <int32_t, Mat> pixelLabels = buildPixelLabels(frame, pixelDistributions); // matrix contains relative estimations that the particular pixel belongs to current bodypart
  Mat maskMat = frame->getMask(); // copy mask from the frame
  
  // For all body parts
  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  { //Temporary variables
    vector <LimbLabel> labels;
    vector <Point2f> uniqueLocations;
    vector <LimbLabel> sortedLabels;
    vector <vector <LimbLabel>> allLabels;
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
    float minDist = boxWidth * 0.2f; // linear step of searching
    if (minDist < 2) minDist = 2; // the minimal linear step
    float searchDistance = 0;
    try
    {      
      float mult = partTree.depth(iteratorBodyPart) * searchDistCoeffMult;
      if (mult == 0) mult = 1;
      searchDistance = boneLength * params.at(sSearchDistCoeff) * partTree.depth(iteratorBodyPart) * mult; // the limiting of search area
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
      minTheta = params.at(sMinTheta); // the start angle for searching
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
      maxTheta = params.at(sMaxTheta);  // the end angle for searching
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
      stepTheta = params.at(sStepTheta); // angular search step
    }
    catch (...)
    {
      stringstream ss;
      ss << "Maybe there is no '" << sStepTheta << "' param";
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
            ss << "Can't get value in maskMat at " << "[" << y << "][" << x << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
          bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
          if (!blackPixel)
          { // Scan the possible rotation zone
            for (float rot = theta - minTheta; rot < theta + maxTheta; rot += stepTheta)
            {
              // Create a new label vector and build it label
              Point2f p0 = Point2f(0, 0); // the point of unit vector
              Point2f p1 = Point2f(1.0, 0); // the point of unit vector
              p1 *= boneLength; // change the vector length 
              p1 = PoseHelper::rotatePoint2D(p1, p0, rot); // rotate the vector
              Point2f mid = 0.5 * p1; // center of the vector
              p1 = p1 + Point2f(x, y) - mid; // shift the vector to current point
              p0 = Point2f(x, y) - mid; // shift the vector to current point
              LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, frame, pixelDistributions, pixelLabels, p0, p1, useCSdet); // build  the vector label
              sortedLabels.push_back(generatedLabel); // add label to current bodypart labels
            }
          }
        }
      }
    }
    if (sortedLabels.size() == 0) // if labels for current body part is not builded
    {
      for (float rot = theta - minTheta; rot < theta + maxTheta; rot += stepTheta)
      {
        Point2f p0 = Point2f(0, 0); // the point of unit vector
        Point2f p1 = Point2f(1.0, 0); // the point of unit vector
        p1 *= boneLength; // change the vector length 
        p1 = PoseHelper::rotatePoint2D(p1, p0, rot); // rotate the vector
        Point2f mid = 0.5 * p1; // center of the vector
        p1 = p1 + Point2f(suggestStart.x, suggestStart.y) - mid; // shift the vector to reference point
        p0 = Point2f(suggestStart.x, suggestStart.y) - mid; // shift the vector to reference point
        LimbLabel generatedLabel = generateLabel(*iteratorBodyPart, frame, pixelDistributions, pixelLabels, p0, p1, useCSdet);
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
      Mat locations(frame->getImage().cols, frame->getImage().rows, DataType<uint32_t>::type); // create the temporary matrix
      for (int32_t i = 0; i < frame->getImage().cols; i++)
      {
        for (int32_t j = 0; j < frame->getImage().rows; j++)
        {
          try
          {
            locations.at<uint32_t>(i, j) = 0; // init elements of the "location" matrix
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
      // For all "sortedLabels"
      for (uint32_t i = 0; i < sortedLabels.size(); i++)
      {
        uint32_t x = (uint32_t)sortedLabels.at(i).getCenter().x; // copy center coordinates of current label
        uint32_t y = (uint32_t)sortedLabels.at(i).getCenter().y; // copy center coordinates of current label
        try
        {
          if (locations.at<uint32_t>(x, y) < uniqueLocationCandidates) // current point is occupied by less then "uniqueLocationCandidates" of labels with a greater score
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
            locations.at<uint32_t>(x, y) += 1; // increase the counter of labels number at given point
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
    t.push_back(labels); // add current point labels
  }
  map <int32_t, Mat>::iterator i;
  for (i = pixelDistributions.begin(); i != pixelDistributions.end(); ++i)
  {
    i->second.release();
  }
  maskMat.release();
  return merge(limbLabels, t);
}

// Return nBins
uint8_t ColorHistDetector::getNBins(void)
{
  return nBins;
}

// Returns relative frequency of the RGB-color reiteration in "PartModel" 
float ColorHistDetector::computePixelBelongingLikelihood(const PartModel &partModel, uint8_t r, uint8_t g, uint8_t b)
{ // Scaling of colorspace, finding the colors interval, which now gets this color
  uint8_t factor = static_cast<uint8_t> (ceil(pow(2, 8) / partModel.nBins));
  float isFG = 0;
  try
  {
    isFG = partModel.partHistogramm.at(r / factor).at(g / factor).at(b / factor); // relative frequency of current color reiteration 
  }
  catch (...)
  {
    stringstream ss;
    ss << "Couldn't find partHistogramm " << "[" << (int)r / factor << "][" << (int)g / factor << "][" << (int)b / factor << "]";
    if (debugLevelParam >= 1)
      cerr << ERROR_HEADER << ss.str() << endl;
    throw logic_error(ss.str());
  }
  return isFG;
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
          ss << "Couldn't find partHistogramm " << "[" << (int)r / factor << "][" << (int)g / factor << "][" << (int)b / factor << "]";
          if (debugLevelParam >= 1)
            cerr << ERROR_HEADER << ss.str() << endl;
          throw logic_error(ss.str());
        }
      }
    }
  }
  uint8_t r, g, b;
  // Scaling of colorspace, reducing the capacity and number of colour intervals that are used to construct the histogram
  for (uint32_t i = 0; i < partColors.size(); i++)
  {
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
        partModel.partHistogramm[r][g][b] *= partModel.sizeFG; // converting the colors relative frequency into the pixels number
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
    uint8_t r = static_cast<uint8_t> (partColors[i].x / factor);
    uint8_t g = static_cast<uint8_t> (partColors[i].y / factor);
    uint8_t b = static_cast<uint8_t> (partColors[i].z / factor);
    partModel.partHistogramm[r][g][b]++; // increment the frequency of interval, that this color have hit
  }

  //renormalise
  for (uint8_t r = 0; r < partModel.nBins; r++)
  {
    for (uint8_t g = 0; g < partModel.nBins; g++)
    {
      for (uint8_t b = 0; b < partModel.nBins; b++)
      {
        //normalise the histograms
        partModel.partHistogramm[r][g][b] /= partModel.sizeFG;
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
    sum += partModel.fgSampleSizes[i];
  }
  sum /= partModel.fgNumSamples;
  return sum;
}

// Averaging the number of samples, that united from two sets
float ColorHistDetector::getAvgSampleSizeFgBetween(const PartModel &partModel, uint32_t s1, uint32_t s2)
{
  if (s1 >= partModel.fgSampleSizes.size() || s2 >= partModel.fgSampleSizes.size())
    return 0;
  return (partModel.fgSampleSizes[s1] + partModel.fgSampleSizes[s2]) / 2.0f;
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
        distance += pow(partModel.partHistogramm[r][g][b] - partModelPrev.partHistogramm[r][g][b], 2);
      }
    }
  }
  float score = sqrt(distance);
  return score;
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
        partModel.bgHistogramm[r][g][b] *= partModel.sizeBG;
      }
    }
  }
  uint32_t factor = (uint32_t)ceil(pow(2, 8) / partModel.nBins); // colorspace scaling coefficient
  partModel.sizeBG += static_cast <uint32_t> (bgColors.size());
  partModel.bgNumSamples++;
  partModel.bgSampleSizes.push_back(static_cast <uint32_t> (bgColors.size()));
  for (uint32_t i = 0; i < bgColors.size(); i++)
  {
      partModel.bgHistogramm[bgColors[i].x / factor][bgColors[i].y / factor][bgColors[i].z / factor]++; // increment the frequency of interval, that this color have hit
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

// Returns a matrix, that contains relative frequency of the pixels colors reiteration 
map <int32_t, Mat> ColorHistDetector::buildPixelDistributions(Frame *frame)
{
  Skeleton skeleton = frame->getSkeleton(); // copy skeleton from the frame
  tree <BodyPart> partTree = skeleton.getPartTree(); // copy part tree from the skeleton
  tree <BodyPart>::iterator iteratorBodyPart;
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
  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    Mat t = Mat(width, height, DataType <float>::type); // create empty matrix
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

          t.at<float>(x, y) = blackPixel ? 0 : computePixelBelongingLikelihood(partModel, red, green, blue); // relative frequency of the current pixel color reiteration 
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
  tree <BodyPart>::iterator iteratorBodyPart;
  map <int32_t, Mat> pixelLabels;
  // For all body parts
  for (iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
  {
    Mat t = Mat(width, height, DataType <float>::type); // create empty matrix
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
            if (temp.at<float>(x, y) > top) // search max value of the current bodypart pixel color frequency
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

LimbLabel ColorHistDetector::generateLabel(BodyPart bodyPart, Frame *frame, map <int32_t, Mat> pixelDistributions, map <int32_t, Mat> pixelLabels, Point2f j0, Point2f j1, float _useCSdet)
{
  vector <Score> s;
  vector <Point3i> partPixelColours;
  Mat maskMat = frame->getMask(); // copy mask from the frame 
  Mat imgMat = frame->getImage(); // copy image from the frame
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5; // segment center
  float boneLength = getBoneLength(j0, j1); // distance between joints
  float rot = float(PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));// tilt angle
  POSERECT <Point2f> rect = getBodyPartRect(bodyPart, j0, j1); // expected bodypart location area?
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
    model = partModels.at(bodyPart.getPartID()); // copy part model for the "bodyPart"
  }
  catch (...)
  {
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
    if (debugLevelParam >= 2)
      cerr << ERROR_HEADER << "Dirty label!" << endl;
    Score sc(0.0, detectorName.str());
    s.push_back(sc);
    return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s, true); // create the limb label
  }
  // Scan the area near the bodypart center
  for (int32_t i = int32_t(boxCenter.x - boneLength * 0.5); i < int32_t(boxCenter.x + boneLength * 0.5); i++)
  {
    for (int32_t j = int32_t(boxCenter.y - boneLength * 0.5); j < int32_t(boxCenter.y + boneLength * 0.5); j++)
    {
      if (i < maskMat.cols && j < maskMat.rows) // if the point is within the image
      {
        float xmax, ymax, xmin, ymin;
        rect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax); // highlight the extreme points of the body part rect

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
              Score sc(0.0, detectorName.str());
              s.push_back(sc);
              return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s, true); // create the limb label
            }
            bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
            if (!blackPixel)
            {
              try
              {
                pixDistAvg += pixelDistributions.at(bodyPart.getPartID()).at<float>(i, j); // Accumulation the "distributions" of contained pixels
              }
              catch (...)
              {
                maskMat.release();
                imgMat.release();
                if (debugLevelParam >= 2)
                  cerr << ERROR_HEADER << "Dirty label!" << endl;
                Score sc(0.0, detectorName.str());
                s.push_back(sc);
                return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s, true); // create the limb label
              }
              pixDistNum++; // counting of the all scanned pixels
              try
              {
                if (pixelLabels.at(bodyPart.getPartID()).at<float>(i, j))
                {
                  pixelsWithLabel++;
                  totalPixelLabelScore += pixelLabels.at(bodyPart.getPartID()).at<float>(i, j); // Accumulation of the pixel labels
                }
              }
              catch (...)
              {
                maskMat.release();
                imgMat.release();
                if (debugLevelParam >= 2)
                  cerr << ERROR_HEADER << "Dirty label!" << endl;
                Score sc(0.0, detectorName.str());
                s.push_back(sc);
                return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s, true); // create the limb label
              }
              // copy colors components of the current pixel
              Vec3b intensity = imgMat.at<Vec3b>(j, i);
              uint8_t blue = intensity.val[0];
              uint8_t green = intensity.val[1];
              uint8_t red = intensity.val[2];
              Point3i ptColor(red, green, blue);
              pixelsInMask++; // counting pixels within the mask
              partPixelColours.push_back(ptColor); // insert part pixel color into colorset
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
  pixDistAvg /= (float)pixDistNum;  // average "distributions"
  float inMaskSuppWeight = 0.5;
  if (partPixelColours.size() > 0)
  {
    supportScore = (float)totalPixelLabelScore / (float)totalPixels;
    inMaskSupportScore = (float)totalPixelLabelScore / (float)pixelsInMask;
    PartModel model(nBins);
    setPartHistogramm(model, partPixelColours); // Build the part histogram
    float score = 1.0f - ((1.0f - inMaskSuppWeight)*supportScore + inMaskSuppWeight*inMaskSupportScore);
    /*if (score < 0)
    {
    stringstream ss;
    ss << "Score can't be less thah zero" << endl;
    #ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
    #endif  // DEBUG
    throw logic_error(ss.str());
    }*/
    Score sc(score, detectorName.str()); // create the score
    s.push_back(sc); // the set of scores
    return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s);// create the limb label
  }
  if (debugLevelParam >= 2)
    cerr << ERROR_HEADER << "Dirty label!" << endl;
  Score sc(0.0, detectorName.str());
  s.push_back(sc);
  return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s, true); // create the limb label
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
