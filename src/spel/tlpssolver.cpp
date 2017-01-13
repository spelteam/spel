// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

// OpenGM
#include <opengm/graphicalmodel/space/discretespace.hxx>
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/graphicalmodel_hdf5.hxx>
#include <opengm/operations/minimizer.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>

#include "lockframe.hpp"

#include "colorHistDetector.hpp"
#include "hogDetector.hpp"
#include "surfDetector.hpp"

#include "tlpssolver.hpp"
#include "spelParameters.hpp"

namespace SPEL
{
  TLPSSolver::TLPSSolver(void)
  {
    m_id = 0;
    m_name = "TLPS";
  }

  TLPSSolver::~TLPSSolver(void)
  {

  }

  std::vector<Solvlet> TLPSSolver::solve(Sequence &frames) //inherited virtual
  {
    std::map<std::string, float> params; //set the default parameters std::

    emplaceDefaultParameters(params);

    return solve(frames, params);
  }

  std::vector<Solvlet> TLPSSolver::solve(Sequence &sequence, std::map<std::string, float> params, const std::vector<Solvlet> &solvlets) //inherited virtual
  {
    emplaceDefaultParameters(params);
    auto tlpsSolvlets = solve(sequence, params); //inherited virtual

    for (const auto& s : solvlets)
      tlpsSolvlets.push_back(s);

    sort(tlpsSolvlets.begin(), tlpsSolvlets.end());

    return tlpsSolvlets;
  }

  std::vector<Solvlet> TLPSSolver::solve(Sequence &sequence, std::map<std::string, float> params) //inherited virtual
  {
    auto frames = sequence.getFrames();
    if (frames.size() == 0)
      return std::vector<Solvlet>();

    emplaceDefaultParameters(params);

    //detector search parameters
    const auto baseRotationRange =
      params.at(COMMON_SOLVER_PARAMETERS::BASE_ROTATION_RANGE().name());
    params.at(COMMON_SOLVER_PARAMETERS::BASE_ROTATION_STEP().name()) =
      baseRotationRange / 4.0f;
    params.at(COMMON_SOLVER_PARAMETERS::BASE_SEARCH_RADIUS().name()) =
      frames.front()->getImageSize().height / 30.0f;
    params.at(COMMON_SOLVER_PARAMETERS::BASE_SEARCH_STEP().name()) =
      params.at(COMMON_SOLVER_PARAMETERS::BASE_SEARCH_RADIUS().name()) / 10.0f;

    sequence.estimateUniformScale(params);
    sequence.computeInterpolation(params);

    for (auto f : frames)
      delete f;

    //call the new function
    if (spelHelper::compareFloat(params.at(
      COMMON_TLPS_SOLVER_PARAMETERS::TEMPORAL_WINDOW_SIZE().name()), 0.0f) == 0)
      return solveGlobal(sequence, params);
    else
      return solveWindowed(sequence, params);
  }

  std::vector<Solvlet> TLPSSolver::solveGlobal(Sequence &sequence, std::map<std::string, float> params) //inherited virtual
  {
    emplaceDefaultParameters(params);

    const auto partShiftCoeff = params.at(
      COMMON_TLPS_SOLVER_PARAMETERS::PART_SHIFT_COEFFICIENT().name());
    const auto partRotationCoeff = params.at(
      COMMON_TLPS_SOLVER_PARAMETERS::PART_ROTATION_COEFFICIENT().name());

    const auto depthRotationCoeff = params.at(
      COMMON_SOLVER_PARAMETERS::PART_DEPTH_ROTATION_COEFFICIENT().name());
    const auto baseRotationRange = params.at(
      COMMON_SOLVER_PARAMETERS::BASE_ROTATION_RANGE().name());
    const auto baseSearchRadius = params.at(
      COMMON_SOLVER_PARAMETERS::BASE_SEARCH_RADIUS().name());

    const auto useHoG = spelHelper::compareFloat(params.at(
      COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR().name()), 0.0f) > 0;
    const auto useCS = spelHelper::compareFloat(params.at(
      COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR().name()), 0.0f) > 0;
    const auto useSURF = spelHelper::compareFloat(params.at(
      COMMON_DETECTOR_PARAMETERS::USE_SURF_DETECTOR().name()), 0.0f) > 0;

    //first slice up the sequences
    if (SpelObject::getDebugLevel() >= 1)
      std::cout << "TLPSSolver started, slicing sequence..." << std::endl;

    auto origFrames = sequence.getFrames();
    auto slices = slice(origFrames);

    if (SpelObject::getDebugLevel() >= 1)
      std::cout << slices.size() << " sequence slices created." << std::endl;

    std::vector<std::vector<Solvlet>> sequenceSolvlets; //one vector of solvlets per slice

    if (SpelObject::getDebugLevel() >= 1)
      std::cout << "Solving slices..." << std::endl;

    for (auto &seqSlice : slices)
    {
      ///define the space
      typedef opengm::DiscreteSpace<> Space;
      ///define the model
      typedef opengm::GraphicalModel<float, opengm::Adder, opengm::ExplicitFunction<float>, Space> Model;
      ///define the update rules
      typedef opengm::BeliefPropagationUpdateRules<Model, opengm::Minimizer> UpdateRules;
      ///define the inference algorithm
      typedef opengm::MessagePassing<Model, opengm::Minimizer, UpdateRules, opengm::MaxDistance> BeliefPropagation;

      const auto maxSeqSize = 100;
      if (seqSlice.size() >= maxSeqSize) //change this to a parameter later
        continue;

      std::vector<Detector*> detectors;
      if (useCS)
        detectors.push_back(new ColorHistDetector());
      if (useHoG)
        detectors.push_back(new HogDetector());
      if (useSURF)
        detectors.push_back(new SurfDetector());

      std::vector<Frame*> trainingFrames;
      trainingFrames.push_back(seqSlice.front()); //set training frame by index
      trainingFrames.push_back(seqSlice.back());

      for (auto &detector : detectors)
        detector->train(trainingFrames, params);

      std::vector<std::map<uint32_t, std::vector<LimbLabel>>> detections; //numbers of labels per part, per frame, for this slice
      detections.reserve(seqSlice.size());

      //first do the detections, and store them

      for (auto &currentFrame : seqSlice) //for every frame but first and last
      {
        //current frame represents the current frame, previous frame is currentFrame-1, next frame is currentFrame+1
        if (SpelObject::getDebugLevel() >= 1)
          std::cerr << "Detecting on frame " << currentFrame->getID() << std::endl;

        std::map<uint32_t, std::vector<LimbLabel> > labels;

        auto skeleton = currentFrame->getSkeleton();

        //now set up skeleton params, such as search radius and angle search radius for every part
        //this should very depending on relative distance between frames
        //for each body part
        auto partTree = skeleton.getPartTree();
        for (auto partIter = partTree.begin(); partIter != partTree.end(); ++partIter)
        {
          //for each bodypart, establish the angle variation and the search distance, based on distance from parent frame
          //and based on node depth (deeper nodes have a higher distance)
          //this should rely on parameters e.g.

          const auto depth = partTree.depth(partIter);
          auto rotationRange = baseRotationRange;
          auto searchRange = baseSearchRadius * pow(depthRotationCoeff, depth);

          if (partTree.number_of_children(partIter) == 0)
          {
            searchRange *= 2;
            rotationRange *= depthRotationCoeff;
          }

          partIter->setRotationSearchRange(rotationRange);
          partIter->setSearchRadius(searchRange);
        }

        skeleton.setPartTree(partTree);
        currentFrame->setSkeleton(skeleton);

        for (const auto &detector : detectors) //for every detector
          labels = detector->detect(currentFrame, params, labels); //detect labels based on keyframe training

        const auto maxPartCandidates = params.at(
          COMMON_SOLVER_PARAMETERS::MAX_PART_CANDIDATES().name());
        //now take the top percentage of all labels
        for (auto &label : labels) //for each part
        {
          const auto scores = label.second.at(0).getScores();

          auto isWeak = 0U;
          for (const auto &score : scores)
            if (score.getIsWeak())
              ++isWeak;

          std::vector<LimbLabel> tmp;
          const auto limit = static_cast<uint32_t>(label.second.size() < maxPartCandidates ?
            label.second.size() : maxPartCandidates);
          for (auto j = 0U; j < limit; ++j) //for each label that is within the threshold
            tmp.push_back(label.second.at(j)); //push back the label
          label.second = tmp; //set this part's candidates to the new trimmed vector
        }

        detections.push_back(labels); //store all detections into detections
      }

      std::vector<size_t> numbersOfLabels; //numbers of labels per part

      //the first an last frames of detections are always empty

      for (const auto &i : detections)
        for (const auto &j : i)
          numbersOfLabels.push_back(j.second.size()); //numbers of labels now contains the numbers of labels, and their respective variations

      //use this to shape the space, this will be rather large
      Space space(numbersOfLabels.begin(), numbersOfLabels.end());
      Model gm(space);

      if (SpelObject::getDebugLevel() >= 1)
        std::cerr << "Computing slice factors..." << std::endl;

      auto suppFactors = 0, jointFactors = 0, anchorFactors = 0, tempFactors = 0;
      //now do the factors
      for (auto currentFrame = 1U; currentFrame < seqSlice.size() - 1; ++currentFrame) //for every frame but first and last
      {
        auto currentFrameSlice = seqSlice.at(currentFrame);
        const auto prevFrameSlice = seqSlice.at(currentFrame - 1);
        auto tempSkel = currentFrameSlice->getSkeleton(); //this frame interpolated skeleton
        const auto pSkel = prevFrameSlice->getSkeleton(); //previous frame interpolated skeleton
        auto partTree = tempSkel.getPartTree();
        const auto prevPartTree = pSkel.getPartTree();

        auto partIter = partTree.begin();
        auto prevPartIter = prevPartTree.begin();
        while (partIter != partTree.end() && prevPartIter != prevPartTree.end())
        {
          //compute prev part angle
          const auto p0 = pSkel.getBodyJoint(prevPartIter->getParentJoint())->getImageLocation();
          const auto p1 = pSkel.getBodyJoint(prevPartIter->getChildJoint())->getImageLocation();

          const auto c0 = tempSkel.getBodyJoint(partIter->getParentJoint())->getImageLocation();
          const auto c1 = tempSkel.getBodyJoint(partIter->getChildJoint())->getImageLocation();
          //compute prev part centre shift
          const auto partCentre = 0.5 * c0 + 0.5 * c1;
          const auto prevPartCentre = 0.5 * p0 + 0.5 * p1;

          const auto partShift = partCentre - prevPartCentre;

          //set the search range based on distance body part centre has moved between previous frame and this frame
          const auto searchRange = sqrt(partShift.x * partShift.x + partShift.y * partShift.y) * partShiftCoeff;

          //bow compute the rotation angle change
          const auto partAngle = spelHelper::angle2D(1.0f, 0.0f, c1.x - c0.x, c1.y - c0.y) * static_cast<float>(180.0f / M_PI);
          const auto prevAngle = spelHelper::angle2D(1.0f, 0.0f, p1.x - p0.x, p1.y - p0.y) * static_cast<float>(180.0f / M_PI);

          //set search radius based on part rotation between previous frame and this frame
          const auto rotationRange = (partAngle - prevAngle) * partRotationCoeff;

          partIter->setRotationSearchRange(rotationRange);
          partIter->setSearchRadius(searchRange);

          ++partIter;
          ++prevPartIter;
        }

        tempSkel.setPartTree(partTree); //set the part tree for the skel
        currentFrameSlice->setSkeleton(tempSkel); //set the skeleton for the frame

        //construct the image score cost factors
        //label score cost
        if (SpelObject::getDebugLevel() >= 1)
          std::cerr << "Computing Factors at Frame " << currentFrameSlice->getID() << std::endl;
        const auto &labels = detections.at(currentFrame);
        for (partIter = partTree.begin(); partIter != partTree.end(); ++partIter) //for each of the detected parts
        {
          const auto partId = partIter->getPartID();
          const auto partTreeSize = partTree.size();
          const auto &partIterLabel = labels.at(partId);
          const auto partIterLabelSize = partIterLabel.size();

          const auto scores = partIterLabel.at(0).getScores();
          auto isWeakCount = 0U;
          for (const auto &score : scores)
            if (score.getIsWeak())
              isWeakCount++;

          std::vector<int> varIndices; //create vector of indices of variables
          const auto varID = (currentFrame - 1) * partTreeSize + partId;
          varIndices.push_back(varID); //push first value in
          const size_t scoreCostShape[] = { partIterLabelSize }; //number of labels

          opengm::ExplicitFunction<float> scoreCostFunc(scoreCostShape, scoreCostShape + 1); //explicit function declare
          for (auto i = 0U; i < partIterLabelSize; ++i) //for each label in for this part
            //compute the label score cost
            scoreCostFunc(i) = computeScoreCost(partIterLabel.at(i), params);

          const auto scoreFid = gm.addFunction(scoreCostFunc); //explicit function add to graphical model
          gm.addFactor(scoreFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
          ++suppFactors;

          if (currentFrame == 1U || currentFrame == (seqSlice.size() - 2))
          {
            const auto currentFrameSlice = currentFrame == 1U ? seqSlice.at(0) : seqSlice.at(seqSlice.size() - 1);
            auto anchorMax = 0.0f;
            for (const auto &i : partIterLabel)
            {
              const auto val = computeAnchorCost(i, currentFrameSlice, params);

              if (val > anchorMax && val != std::numeric_limits<float>::max())
                anchorMax = val;
            }

            //we already know the shape from the previous functions
            opengm::ExplicitFunction<float> anchorCostFunc(scoreCostShape, scoreCostShape + 1); //explicit function declare

            for (auto i = 0U; i < partIterLabelSize; ++i) //for each label in for this part
              anchorCostFunc(i) = computeNormAnchorCost(partIterLabel.at(i), currentFrameSlice, params, anchorMax);;

            gm.addFactor(gm.addFunction(anchorCostFunc), varIndices.begin(), varIndices.end()); //bind to factor and variables
            ++anchorFactors;
          }

          if (partIter != partTree.begin()) //if iterator is not on root node, there is always a parent body part
          {
            //this factor needs to be in reverse order, for sorting purposes
            varIndices.clear();
            const auto parentPartIter = partTree.parent(partIter); //find the parent of this part
            const auto parentPartIterId = parentPartIter->getPartID();
            varIndices.push_back((currentFrame - 1) * partTree.size() + parentPartIter->getPartID()); //push back parent partID as the second variable index
            varIndices.push_back(varID); //push first value in (parent, this)

            size_t jointCostShape[] = { partIterLabelSize, partIterLabelSize }; //number of labels
            opengm::ExplicitFunction<float> jointCostFunc(jointCostShape, jointCostShape + 2); //explicit function declare

            //first figure out which of the current body part's joints should be in common with the parent body part

            const bool toChild = parentPartIter->getChildJoint() == partIter->getParentJoint();
            float jointMin = FLT_MAX, jointMax = 0;
            for (uint32_t i = 0; i < labels.at(partIter->getPartID()).size(); ++i) //for each label in for this part
            {
              for (uint32_t j = 0; j < labels.at(parentPartIter->getPartID()).size(); ++j)
              {
                float val = computeJointCost(labels.at(partIter->getPartID()).at(i), labels.at(parentPartIter->getPartID()).at(j), params, toChild);

                if (val < jointMin)
                  jointMin = val;
                if (val > jointMax && val != FLT_MAX)
                  jointMax = val;
              }
            }

            for (uint32_t i = 0; i < labels.at(partIter->getPartID()).size(); ++i) //for each label in for this part
            {
              for (uint32_t j = 0; j < labels.at(parentPartIter->getPartID()).size(); ++j)
              {
                //for every child/parent pair, compute score
                float cost = computeNormJointCost(labels.at(partIter->getPartID()).at(i), labels.at(parentPartIter->getPartID()).at(j), params, jointMax, toChild);
                jointCostFunc(j, i) = cost;
              }
            }

            Model::FunctionIdentifier jointFid = gm.addFunction(jointCostFunc); //explicit function add to graphical model
            gm.addFactor(jointFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
            jointFactors++;
          }

          //futre temporal link (if not anchored)
          if (currentFrame < seqSlice.size() - 2)
          {

            float tempMin = FLT_MAX, tempMax = 0;
            for (uint32_t i = 0; i < labels.at(partIter->getPartID()).size(); ++i) //for each label in for this part
            {
              for (uint32_t j = 0; j < detections[currentFrame + 1][partIter->getPartID()].size(); ++j)
              {
                float val = computeFutureTempCost(labels.at(partIter->getPartID()).at(i), detections[currentFrame + 1][partIter->getPartID()].at(j), params);

                if (val < tempMin)
                  tempMin = val;
                if (val > tempMax && val != FLT_MAX)
                  tempMax = val;
              }
            }

            varIndices.clear();
            varIndices.push_back((currentFrame - 1)*partTree.size() + partIter->getPartID()); //push back parent partID as the second variable index
            varIndices.push_back((currentFrame)*partTree.size() + partIter->getPartID());

            size_t futureCostShape[] = { labels.at(partIter->getPartID()).size(), detections[currentFrame + 1][partIter->getPartID()].size() }; //number of labels
            opengm::ExplicitFunction<float> futureTempCostFunc(futureCostShape, futureCostShape + 2); //explicit function declare

            for (uint32_t i = 0; i < labels.at(partIter->getPartID()).size(); ++i) //for each label in for this part
            {
              for (uint32_t j = 0; j < detections[currentFrame + 1][partIter->getPartID()].size(); ++j)
              {
                float cost = computeNormFutureTempCost(labels.at(partIter->getPartID()).at(i), detections[currentFrame + 1][partIter->getPartID()].at(j), params, tempMax);
                futureTempCostFunc(i, j) = cost;
              }
            }
            Model::FunctionIdentifier futureTempCostFid = gm.addFunction(futureTempCostFunc); //explicit function add to graphical model
            gm.addFactor(futureTempCostFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
            tempFactors++;
          }
        }
      }

      if (SpelObject::getDebugLevel() > 0)
      {
        float n, k;
        n = seqSlice.size() - 2; //num non-anchor frames
        k = seqSlice.front()->getSkeleton().getPartTree().size(); //number of bones
        //float expectedSuppFactors = n*k; //n*k
        float expectedJointFactors = n*(k - 1); //n*(k-1)
        float expectedAnchorFactors = 2 * k; //2*k
        float expectedTempFactors = (n - 1)*k; //(n-1)*k

        //assert(expectedSuppFactors == suppFactors);
        assert(expectedJointFactors == jointFactors);
        assert(expectedAnchorFactors == anchorFactors);
        assert(expectedTempFactors == tempFactors);
      }

      std::cerr << "Solving slice factor graph" << std::endl;

      //now solve the slice

      if (SpelObject::getDebugLevel() >= 2)
        opengm::hdf5::save(gm, "gm.h5", "tlps-gm");

      const size_t maxNumberOfIterations = 100;
      const double convergenceBound = 1e-7;
      const double damping = 0.5;
      BeliefPropagation::Parameter parameter(maxNumberOfIterations, convergenceBound, damping);
      BeliefPropagation bp(gm, parameter);

      // optimize (approximately)
      BeliefPropagation::VerboseVisitorType visitor;
      //bp.infer(visitor);
      bp.infer();
      //pass to solve function

      // obtain the (approximate) argmin
      std::vector<size_t> labeling((detections.size() - 2)*detections[1].size()); //number of non-anchor frames * number of bodyparts
      bp.arg(labeling);

      std::vector<std::vector<LimbLabel> > solutionLabels;
      for (uint32_t i = 0; i < seqSlice.size(); ++i)
        solutionLabels.push_back(std::vector<LimbLabel>());

      for (uint32_t i = 1; i < seqSlice.size() - 1; ++i) //frames
      {
        for (uint32_t j = 0; j < detections[i].size(); ++j) //parts
        {
          int solveId = (i - 1)*detections[1].size() + j; //where to find the solution?
          solutionLabels[i].push_back(detections[i][j][labeling[solveId]]); //pupulate solution vector
        }
      }

      std::vector<Solvlet> solvlets;
      //now set up a solvlet for every frame
      for (uint32_t i = 1; i < seqSlice.size() - 1; ++i) //push all frames but keyframes into the solvlets
        solvlets.push_back(Solvlet(seqSlice[i]->getID(), solutionLabels[i]));

      sequenceSolvlets.push_back(solvlets);

      //do detector cleanup
      for (auto i = 0; i < detectors.size(); ++i)
        delete detectors[i];
      detectors.clear();
      //slice clean-up
//        for(auto f:slices[sliceNumber])
//            delete f;
//        slices[sliceNumber].clear();
    }
    //    slices.clear();

    if (SpelObject::getDebugLevel() >= 1)
      std::cout << sequenceSolvlets.size() << " slices solved." << std::endl;
    //sequence solvlets now contain all the solvlets for the entire sequence

    //rearrange this into one vector of solvlets for evaluation
    std::vector<Solvlet> retSolve;
    for (uint32_t i = 0; i < sequenceSolvlets.size(); ++i)
      for (uint32_t j = 0; j < sequenceSolvlets[i].size(); ++j)
        retSolve.push_back(sequenceSolvlets[i][j]);

    std::vector<Frame*> frames = sequence.getFrames();

    auto acceptLockframeThreshold = params.at(
      COMMON_TLPS_SOLVER_PARAMETERS::LOCKFRAME_THRESHOLD().name());
    if (acceptLockframeThreshold > 0) //if it's greater than zero, we want to evaluate solves before returning them
    {
      std::vector<Solvlet> passedSolves;
      for (uint32_t i = 0; i < retSolve.size(); ++i)
      {
        Solvlet solvlet = retSolve[i];
        float score = evaluateSolution(frames[retSolve[i].getFrameID()],
          solvlet.getLabels(), params);

        if (score >= acceptLockframeThreshold) //if the frame passed the test
        {
          passedSolves.push_back(solvlet);
          //generate lockframe

          //set lockframe to frame
          int thisFrameID = solvlet.getFrameID();
          int parentFrameID = thisFrameID;//this frame is it's own parent frame, since this is a temporal solve

          Lockframe * lockframe = new Lockframe();
          lockframe->SetImageFromPath(frames[thisFrameID]->GetImagePath());
          lockframe->SetMaskFromPath(frames[thisFrameID]->GetMaskPath());
          lockframe->setID(thisFrameID);
          Skeleton parentSkel = frames[parentFrameID]->getSkeleton(); //use skeleton to mimic the structure of
          Skeleton skel = solvlet.toSkeleton(parentSkel);
          //parent skeleton is the basis, this will copy scale and other params, but have different locations
          lockframe->setSkeleton(skel);
          lockframe->setParentFrameID(parentFrameID);

          delete frames[thisFrameID]; //remove the existing frame
          frames[thisFrameID] = lockframe;
        }
      }
      retSolve = passedSolves; //return these solves
    }

    //cout << "ALL GOOD" << endl;
    sequence.setFrames(frames);
    for (auto f : frames) //clean up frames
      delete f;
    //cout << "ALL GOOD 2" << endl;
    for (auto f : origFrames)
      delete f;
    //cout << "ALL GOOD 3" << endl;
    slices.clear();
    return retSolve;
  }

  std::vector<Solvlet> TLPSSolver::solveWindowed(Sequence &sequence, std::map<std::string, float> params) //inherited virtual
  {
#ifndef _MSC_VER
    #warning "TLPSSolver::solveWindowed is not implemented"
#else
#pragma message ("TLPSSolver::solveWindowed is not implemented")
#endif
    throw std::logic_error("TLPSSolver::solveWindowed is not implemented");
  }

  float TLPSSolver::evaluateSolution(Frame* frame, std::vector<LimbLabel> labels, std::map<std::string, float> params)
  {
    emplaceDefaultParameters(params);

    auto maxFrameHeight = params.at(
      COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().name());

    cv::Mat mask = frame->getMask().clone();

    float factor = 1;
    //compute the scaling factor
    if (maxFrameHeight != 0)
    {
      factor = (float)maxFrameHeight / (float)mask.rows;

      resize(mask, mask, cvSize(mask.cols * factor, mask.rows * factor));
    }
    for (std::vector<LimbLabel>::iterator label = labels.begin(); label != labels.end(); ++label)
      label->Resize(factor);

    int correctPixels = 0, incorrectPixels = 0;
    int pixelsInMask = 0;
    int coveredPixelsInMask = 0;
    int incorrectlyCoveredPixels = 0;
    int missedPixels = 0;

    for (int i = 0; i < mask.cols; ++i) //at every col - x
    {
      for (int j = 0; j < mask.rows; ++j) //and every row - y
      {
        //int test = labels[0].containsPoint(Point2f(480,100));
        //check whether pixel hit a label from solution
        bool labelHit = false;
        for (std::vector<LimbLabel>::iterator label = labels.begin(); label != labels.end(); ++label)
        {
          if (label->containsPoint(cv::Point2f(i, j))) //this is done in x,y coords
          {
            labelHit = true;
            //break;
          }
        }

        //check pixel colour
        int intensity = mask.at<uchar>(j, i); //this is done with reve
        bool blackPixel = (intensity < 10);

        if (!blackPixel)
          pixelsInMask++;

        if (blackPixel && labelHit) //if black in label, incorrect
        {
          incorrectPixels++;
          incorrectlyCoveredPixels++;
        }
        else if (!blackPixel && !labelHit) //if white not in label, incorret
        {
          incorrectPixels++;
          missedPixels++;
        }
        else if (!blackPixel && labelHit)//otherwise correct
        {
          correctPixels++;
          coveredPixelsInMask++;
        }
        //            else //black pixel and not label hit
        //                correctPixels++; //don't count these at all?
      }
    }

    double solutionEval = (float)correctPixels / ((float)correctPixels + (float)incorrectPixels);

    //now check for critical part failures - label mostly outside of mask

    std::vector<cv::Point2f> badLabelScores;
    auto badLabelThresh = params.at(
      COMMON_SOLVER_PARAMETERS::BAD_LABEL_THRESH().name());

    for (std::vector<LimbLabel>::iterator label = labels.begin(); label != labels.end(); ++label)
    {
      std::vector<cv::Point2f> poly = label->getPolygon(); //get the label polygon
      //compute min and max x and y
      //float xMin, xMax, yMin, yMax;
      std::vector<float> xS = { poly[0].x, poly[1].x, poly[2].x, poly[3].x };
      std::vector<float> yS = { poly[0].y, poly[1].y, poly[2].y, poly[3].y };
      auto xMin = min_element(xS.begin(), xS.end());
      auto xMax = max_element(xS.begin(), xS.end());

      auto yMin = min_element(yS.begin(), yS.end());
      auto yMax = max_element(yS.begin(), yS.end());

      int labelPixels = 0;
      int badLabelPixels = 0;

      for (int x = *xMin; x < *xMax; ++x)
      {
        for (int y = *yMin; y < *yMax; ++y)
        {
          if (label->containsPoint(cv::Point2f(x, y)))
          {
            int intensity = mask.at<uchar>(y, x); //this is done with reverse y,x
            bool blackPixel = (intensity < 10);
            labelPixels++;
            if (blackPixel)
              ++badLabelPixels;
          }
        }
      }

      float labelRatio = 1.0 - (float)badLabelPixels / (float)labelPixels; //high is good

      if (labelRatio < badLabelThresh /*&& !label->getIsWeak()*/ && !label->getIsOccluded()) //not weak, not occluded, badly localised
        badLabelScores.push_back(cv::Point2f(label->getLimbID(), labelRatio));
    }

    if (SpelObject::getDebugLevel() >= 1)
    {
      for (std::vector<cv::Point2f>::iterator badL = badLabelScores.begin(); badL != badLabelScores.end(); ++badL)
      {
        std::cerr << "Part " << badL->x << " is badly localised, with score " << badL->y << std::endl;
      }
    }

    if (badLabelScores.size() != 0) //make the solution eval fail if a part is badly localised
      solutionEval = solutionEval - 1.0;

    if (SpelObject::getDebugLevel() >= 1)
      std::cerr << "Solution evaluation score - " << solutionEval << " for frame " << frame->getID() << " solve from " << frame->getParentFrameID() << std::endl;

    frame->UnloadAll();

    return solutionEval;
  }

  //compute label score
  float TLPSSolver::computeScoreCost(const LimbLabel& label, std::map<std::string, float> params)
  {
    emplaceDefaultParameters(params);

    std::string hogName = "18500";
    std::string csName = "4409412";
    std::string surfName = "21316";

    //@FIX
    auto useHoG =
      params.at(COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR().name());
    auto useCS =
      params.at(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR().name());
    auto useSURF =
      params.at(COMMON_DETECTOR_PARAMETERS::USE_SURF_DETECTOR().name());

    //TODO: Fix score combinations
    std::vector<Score> scores = label.getScores();

    //compute the weighted sum of scores
    float finalScore = 0;
    bool hogFound = false;
    bool csFound = false;
    bool surfFound = false;
    for (uint32_t i = 0; i < scores.size(); ++i)
    {
      float score = scores[i].getScore();
      if (scores[i].getScore() == -1)//if score is -1, set it to 1
      {
        score = 1.0; //set a high cost for invalid scores
      }
      if (scores[i].getDetName() == hogName)
      {
        finalScore = finalScore + score*useHoG;
        hogFound = true;
      }
      else if (scores[i].getDetName() == csName)
      {
        finalScore = finalScore + score*useCS;
        csFound = true;
      }
      else if (scores[i].getDetName() == surfName)
      {
        finalScore = finalScore + score*useSURF;
        surfFound = true;
      }
    }

    //now add 1.0*coeff for each not found score in this label, that should have been there (i.e., assume the worst)
    finalScore += 1.0*useHoG*(!hogFound) + 1.0*useCS*(!csFound) + 1.0*useSURF*(!surfFound);

    return finalScore;
  }

  //compute distance to parent limb label
  float TLPSSolver::computeJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, bool toChild)
  {
    emplaceDefaultParameters(params);
    //emplace default
    //params.emplace("jointCoeff", 0.5);
    //float lambda = params.at("jointCoeff");
    cv::Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    child.getEndpoints(c0, c1);
    parent.getEndpoints(p0, p1);

    //normalise this?

    //return the squared distance from the lower parent joint p1, to the upper child joint c0
    if (toChild)
      return sqrt(pow((c0.x - p1.x), 2) + pow((c0.y - p1.y), 2));
    else //otherwise we're connected to the parent's parent body part
      return sqrt(pow((c0.x - p0.x), 2) + pow((c0.y - p0.y), 2));
  }

  float TLPSSolver::computeNormJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, float max, bool toChild)
  {
    emplaceDefaultParameters(params);
    //read params
    float lambda = params.at(COMMON_SOLVER_PARAMETERS::JOINT_COEFFICIENT().name());

    //float leeway = params.at("jointLeeway");
    cv::Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    child.getEndpoints(c0, c1);
    parent.getEndpoints(p0, p1);

    //normalise this?
    //give some leeway
    float score = 0;
    if (toChild) //connected to parent's child joint
      score = sqrt(pow(c0.x - p1.x, 2) + pow(c0.y - p1.y, 2)) / max;
    else //else connected to parent's parent joint
      score = sqrt(pow(c0.x - p0.x, 2) + pow(c0.y - p0.y, 2)) / max;
    //return the squared distance from the lower parent joint p1, to the upper child joint c0

    //output a sentence about who connected to whom and what the score was
    if (SpelObject::getDebugLevel() >= 1 && (child.getLimbID() == 7 || child.getLimbID() == 6) && !toChild)
      std::cerr << "Part " << child.getLimbID() << " is connecting to part " << parent.getLimbID() << " PARENT joint" << std::endl;

    return lambda*score;
  }

  float TLPSSolver::computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params)
  {
    emplaceDefaultParameters(params);
    //  params.emplace("priorCoeff", 0.0);
    //	float lambda = params.at("priorCoeff");
    cv::Point2f p0, p1, pp0, pp1;
    label.getEndpoints(p0, p1);
    pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
    pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

    //normalise this cost?

    //return the sum of squared distances between the corresponding joints, prior to label
    return pow((p0.x - pp0.x), 2) + pow((p0.y - pp0.y), 2) + pow((p1.x - pp1.x), 2) + pow((p1.y - pp1.y), 2);
  }

  float TLPSSolver::computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params, float max)
  {
    emplaceDefaultParameters(params);

    auto lambda = params.at(COMMON_SOLVER_PARAMETERS::PRIOR_COEFFICIENT().name());
    cv::Point2f p0, p1, pp0, pp1;
    label.getEndpoints(p0, p1);
    pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
    pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

    //normalise this cost?

    //return the sum of squared distances between the corresponding joints, prior to label
    return lambda*((pow((p0.x - pp0.x), 2) + pow((p0.y - pp0.y), 2) + pow((p1.x - pp1.x), 2) + pow((p1.y - pp1.y), 2)) / max);
  }

  float TLPSSolver::computePastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, std::map<std::string, float> params)
  {
    emplaceDefaultParameters(params);
    //compute the temporal connection cost to label in the past

    //@PARAM there should be a parameter that sets the temporal cost constant (beta)
    //compute the joint joining cost for the skeleton
    cv::Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    thisLabel.getEndpoints(c0, c1);
    pastLabel.getEndpoints(p0, p1);

    //return the squared distance from the lower parent joint p1, to the upper child joint c0
    return (pow((c0.x - p0.x), 2) + pow((c0.y - p0.y), 2) + pow((c1.x - p1.x), 2) + pow((c1.y - p1.y), 2));
  }

  float TLPSSolver::computeNormPastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, std::map<std::string, float> params, float max)
  {
    emplaceDefaultParameters(params);
    //emplace default
    auto lambda = params.at(
      COMMON_TLPS_SOLVER_PARAMETERS::TEMPORAL_LINK_COEFFICIENT().name());
    //compute the temporal connection cost to label in the past

    //@PARAM there should be a parameter that sets the temporal cost constant (beta)
    //compute the joint joining cost for the skeleton
    cv::Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    thisLabel.getEndpoints(c0, c1);
    pastLabel.getEndpoints(p0, p1);

    //return the squared distance from the lower parent joint p1, to the upper child joint c0
    return lambda*((pow((c0.x - p0.x), 2) + pow((c0.y - p0.y), 2) + pow((c1.x - p1.x), 2) + pow((c1.y - p1.y), 2)) / max);
  }

  float TLPSSolver::computeFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, std::map<std::string, float> params)
  {
    emplaceDefaultParameters(params);
    //compute temporal connection cost to label in the future
    //@PARAM there needs to be a beta param here as well
    cv::Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    thisLabel.getEndpoints(c0, c1);
    futureLabel.getEndpoints(p0, p1);

    float score = (pow((c0.x - p0.x), 2) + pow((c0.y - p0.y), 2) + pow((c1.x - p1.x), 2) + pow((c1.y - p1.y), 2));
    return score;
  }

  float TLPSSolver::computeNormFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, std::map<std::string, float> params, float max)
  {
    emplaceDefaultParameters(params);
    //emplace default
    float lambda = params.at(
      COMMON_TLPS_SOLVER_PARAMETERS::TEMPORAL_LINK_COEFFICIENT().name());
    //compute temporal connection cost to label in the future
    //@PARAM there needs to be a beta param here as well
    cv::Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    thisLabel.getEndpoints(c0, c1);
    futureLabel.getEndpoints(p0, p1);

    float score = lambda*((pow((c0.x - p0.x), 2) + pow((c0.y - p0.y), 2) + pow((c1.x - p1.x), 2) + pow((c1.y - p1.y), 2)) / max);
    //return the squared distance from the lower parent joint p1, to the upper child joint c0

    return score;
  }

  float TLPSSolver::computeAnchorCost(const LimbLabel& thisLabel, Frame* anchor, std::map<std::string, float> params)
  {
    emplaceDefaultParameters(params);
    //emploace default

    int limbId = thisLabel.getLimbID();

    std::vector<cv::Point2f> partPolygon = anchor->getPartPolygon(limbId);
    std::vector<cv::Point2f> labelPolygon = thisLabel.getPolygon();

    assert(partPolygon.size() == labelPolygon.size());

    //return the error between these'
    float score = 0;
    for (size_t i = 0; i < partPolygon.size(); ++i)
      score += pow(partPolygon[i].x - labelPolygon[i].x, 2) + pow(partPolygon[i].y - labelPolygon[i].y, 2);

    return score;
    //compute the cost of anchoring this label
  }

  float TLPSSolver::computeNormAnchorCost(const LimbLabel& thisLabel, Frame* anchor, std::map<std::string, float> params, float max)
  {
    emplaceDefaultParameters(params);
    //emploace default
    auto lambda =
      params.at(COMMON_TLPS_SOLVER_PARAMETERS::ANCHOR_COEFICIENT().name());

    int limbId = thisLabel.getLimbID();

    std::vector<cv::Point2f> partPolygon = anchor->getPartPolygon(limbId);
    std::vector<cv::Point2f> labelPolygon = thisLabel.getPolygon();

    assert(partPolygon.size() == labelPolygon.size());

    //return the error between these'
    float score = 0;
    for (size_t i = 0; i < partPolygon.size(); ++i)
      score += pow(partPolygon[i].x - labelPolygon[i].x, 2) + pow(partPolygon[i].y - labelPolygon[i].y, 2);

    return lambda*score / max;
    //compute the cost of anchoring this label
  }

  std::vector<std::vector<Frame*> > TLPSSolver::slice(const std::vector<Frame*>& frames) //separate the sequence into slices, for temporal solve
  {
    //frames should be sliced into frame sets, where every non Keyframe non Lockframe frame should belong to a BOUNDED set
    //unbounded sets are not included in the solve
    std::vector<std::vector<Frame*> > aux;
    std::vector<std::vector<Frame*> > slices;

    std::vector<Frame*> currentSet;
    //bool isOpen;
    for (uint32_t i = 0; i < frames.size(); ++i)
    {
      currentSet.push_back(frames[i]); //push the frame to current set
      if (frames[i]->getFrametype() == KEYFRAME || frames[i]->getFrametype() == LOCKFRAME)
      {
        aux.push_back(currentSet);
        currentSet.clear();
        currentSet.push_back(frames[i]);
      }
    }
    //now go through every set, and eliminate it if:
    //1) it contains 2 or less elements
    //2) it doesn't end with a LOCKFRAME or a KEYFRAME
    //3) it doesn't begin with a LOCKFRAME or a KEYFRAME
    for (uint32_t i = 0; i < aux.size(); ++i)
    {
      if (aux[i].at(0)->getFrametype() == LOCKFRAME || aux[i].at(0)->getFrametype() == KEYFRAME) //if the set STARTS with a keyframe or a lockframe
      {
        if (aux[i].back()->getFrametype() == LOCKFRAME || aux[i].back()->getFrametype() == KEYFRAME) //if the set ENDS with a keyframe or a lockframe
        {
          if (aux[i].size() > 2) //if size is greater than two elements
            slices.push_back(aux[i]); //push back slice
        }
      }
    }

    return slices;
  }

  void TLPSSolver::emplaceDefaultParameters(std::map<std::string, float>& params) const
  {
    Solver::emplaceDefaultParameters(params);
    spelHelper::mergeParameters(params, COMMON_TLPS_SOLVER_PARAMETERS::getParameters());
  }

  }
