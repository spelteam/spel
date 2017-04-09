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

    auto frames = sequence.getFrames();
    auto slices = slice(frames);

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

        for (auto iter = partTree.begin(), prevIter = prevPartTree.begin();
          iter != partTree.end() && prevIter != prevPartTree.end(); ++iter, ++prevIter)
        {
          //compute prev part angle
          const auto p0 = pSkel.getBodyJoint(prevIter->getParentJoint())->getImageLocation();
          const auto p1 = pSkel.getBodyJoint(prevIter->getChildJoint())->getImageLocation();

          const auto c0 = tempSkel.getBodyJoint(iter->getParentJoint())->getImageLocation();
          const auto c1 = tempSkel.getBodyJoint(iter->getChildJoint())->getImageLocation();
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

          iter->setRotationSearchRange(rotationRange);
          iter->setSearchRadius(searchRange);
        }

        tempSkel.setPartTree(partTree); //set the part tree for the skel
        currentFrameSlice->setSkeleton(tempSkel); //set the skeleton for the frame

        //construct the image score cost factors
        //label score cost
        if (SpelObject::getDebugLevel() >= 1)
          std::cerr << "Computing Factors at Frame " << currentFrameSlice->getID() << std::endl;
        const auto &labels = detections.at(currentFrame);
        for (auto partIter = partTree.begin(); partIter != partTree.end(); ++partIter) //for each of the detected parts
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

          std::vector<size_t> varIndices; //create vector of indices of variables
          const auto varID = (currentFrame - 1) * partTreeSize + partId;
          varIndices.push_back(varID); //push first value in
          const size_t scoreCostShape[] = { partIterLabelSize }; //number of labels

          opengm::ExplicitFunction<float> scoreCostFunc(scoreCostShape, scoreCostShape + 1); //explicit function declare
          for (auto i = 0U; i < partIterLabelSize; ++i) //for each label in for this part
            //compute the label score cost
            scoreCostFunc(i) = computeScoreCost(partIterLabel.at(i), params);

          gm.addFactor(gm.addFunction(scoreCostFunc), varIndices.begin(), varIndices.end()); //bind to factor and variables
          ++suppFactors;

          if (currentFrame == 1U || currentFrame == (seqSlice.size() - 2))
          {
            const auto slice = currentFrame == 1U ? seqSlice.at(0) : seqSlice.at(seqSlice.size() - 1);
            auto anchorMax = 0.0f;
            for (const auto &i : partIterLabel)
            {
              const auto val = computeAnchorCost(i, slice, params);

              if (val > anchorMax && val != std::numeric_limits<float>::max())
                anchorMax = val;
            }

            //we already know the shape from the previous functions
            opengm::ExplicitFunction<float> anchorCostFunc(scoreCostShape, scoreCostShape + 1); //explicit function declare

            for (auto i = 0U; i < partIterLabelSize; ++i) //for each label in for this part
              anchorCostFunc(i) = computeNormAnchorCost(partIterLabel.at(i), slice, params, anchorMax);;

            gm.addFactor(gm.addFunction(anchorCostFunc), varIndices.begin(), varIndices.end()); //bind to factor and variables
            ++anchorFactors;
          }

          if (partIter != partTree.begin()) //if iterator is not on root node, there is always a parent body part
          {
            //this factor needs to be in reverse order, for sorting purposes
            varIndices.clear();
            const auto parentPartIter = partTree.parent(partIter); //find the parent of this part
            const auto parentPartIterId = parentPartIter->getPartID();
            const auto &parentPartIterLabel = labels.at(parentPartIterId);
            const auto parentPartIterLabelSize = parentPartIterLabel.size();
            varIndices.push_back((currentFrame - 1) * partTree.size() + parentPartIter->getPartID()); //push back parent partID as the second variable index
            varIndices.push_back(varID); //push first value in (parent, this)

            size_t jointCostShape[] = { partIterLabelSize, partIterLabelSize }; //number of labels
            opengm::ExplicitFunction<float> jointCostFunc(jointCostShape, jointCostShape + 2); //explicit function declare

            //first figure out which of the current body part's joints should be in common with the parent body part

            const auto toChild = parentPartIter->getChildJoint() == partIter->getParentJoint();
            auto jointMax = 0.0f;
            for (const auto &i : partIterLabel)
            {
              for (const auto &j : parentPartIterLabel)
              {
                const auto val = computeJointCost(i, j, params, toChild);
                if (val > jointMax && val != std::numeric_limits<float>::max())
                  jointMax = val;
              }
            }

            for (auto i = 0U; i < partIterLabelSize; ++i) //for each label in for this part
            {
              for (auto j = 0U; j < parentPartIterLabelSize; ++j)
              {
                //for every child/parent pair, compute score
                jointCostFunc(j, i) = computeNormJointCost(partIterLabel.at(i), parentPartIterLabel.at(j), params, jointMax, toChild);
              }
            }

            gm.addFactor(gm.addFunction(jointCostFunc), varIndices.begin(), varIndices.end()); //bind to factor and variables
            ++jointFactors;
          }

          //futre temporal link (if not anchored)
          if (currentFrame < seqSlice.size() - 2)
          {
            auto tempMax = 0.0f;
            const auto &currentDetections = detections.at(currentFrame + 1).at(partId);
            const auto currentDetectionsSize = currentDetections.size();
            for (const auto &i : partIterLabel)
            {
              for (const auto &j : currentDetections)
              {
                const auto val = computeFutureTempCost(i, j, params);

                if (val > tempMax && val != std::numeric_limits<float>::max())
                  tempMax = val;
              }
            }

            varIndices.clear();
            varIndices.push_back((currentFrame - 1) * partTree.size() + partId); //push back parent partID as the second variable index
            varIndices.push_back((currentFrame)* partTree.size() + partId);

            size_t futureCostShape[] = { partIterLabelSize, currentDetectionsSize }; //number of labels
            opengm::ExplicitFunction<float> futureTempCostFunc(futureCostShape, futureCostShape + 2); //explicit function declare

            for (auto i = 0U; i < partIterLabelSize; ++i) //for each label in for this part
            {
              for (auto j = 0U; j < currentDetectionsSize; ++j)
              {
                futureTempCostFunc(i, j) = computeNormFutureTempCost(partIterLabel.at(i), currentDetections.at(j), params, tempMax);
              }
            }
            gm.addFactor(gm.addFunction(futureTempCostFunc), varIndices.begin(), varIndices.end()); //bind to factor and variables
            ++tempFactors;
          }
        }
      }

      if (SpelObject::getDebugLevel() > 0)
      {
        const auto n = seqSlice.size() - 2; //num non-anchor frames
        const auto k = seqSlice.front()->getSkeleton().getPartTree().size(); //number of bones
        const auto expectedJointFactors = n * (k - 1); //n*(k-1)
        const auto expectedAnchorFactors = 2 * k; //2*k
        const auto expectedTempFactors = (n - 1) * k; //(n-1)*k

        if (expectedJointFactors != jointFactors)
        {
          std::stringstream ss;
          ss << "expectedJointFactors != jointFactors : " << expectedJointFactors << " != " << jointFactors;
          DebugMessage(ss.str(), 0);
        }
        if (expectedAnchorFactors != anchorFactors)
        {
          std::stringstream ss;
          ss << "expectedAnchorFactors != anchorFactors : " << expectedAnchorFactors << " != " << anchorFactors;
          DebugMessage(ss.str(), 0);
        }
        if (expectedTempFactors != tempFactors)
        {
          std::stringstream ss;
          ss << "expectedTempFactors != tempFactors : " << expectedTempFactors << " != " << tempFactors;
          DebugMessage(ss.str(), 0);
        }
      }

      //now solve the slice

      if (SpelObject::getDebugLevel() >= 2)
        opengm::hdf5::save(gm, "gm.h5", "tlps-gm");

      const auto maxNumberOfIterations = 100U;
      const auto convergenceBound = 1e-7f;
      const auto damping = 0.5f;
      BeliefPropagation bp(gm, BeliefPropagation::Parameter(maxNumberOfIterations, convergenceBound, damping));
      bp.infer();
      //pass to solve function

      // obtain the (approximate) argmin
      const auto firstDetectionsSize = detections.at(1).size();
      std::vector<size_t> labeling((detections.size() - 2) * firstDetectionsSize); //number of non-anchor frames * number of bodyparts
      bp.arg(labeling);

      std::vector<std::vector<LimbLabel> > solutionLabels;
      for (auto i = 0U; i < seqSlice.size(); ++i)
        solutionLabels.push_back(std::vector<LimbLabel>());

      for (auto i = 1U; i < seqSlice.size() - 1; ++i) //frames
      {
        auto &currentDetections = detections.at(i);
        for (auto j = 0U; j < currentDetections.size(); ++j) //parts
        {
          const auto solveId = (i - 1) * firstDetectionsSize + j; //where to find the solution?
          solutionLabels.at(i).push_back(currentDetections.at(j).at(labeling.at(solveId))); //pupulate solution vector
        }
      }

      std::vector<Solvlet> solvlets;
      //now set up a solvlet for every frame
      for (auto i = 1U; i < seqSlice.size() - 1; ++i) //push all frames but keyframes into the solvlets
        solvlets.push_back(Solvlet(seqSlice.at(i)->getID(), solutionLabels.at(i)));

      sequenceSolvlets.push_back(solvlets);

      //do detector cleanup
      for (auto i = 0; i < detectors.size(); ++i)
        delete detectors[i];
      detectors.clear();
    }

    if (SpelObject::getDebugLevel() >= 1)
      std::cout << sequenceSolvlets.size() << " slices solved." << std::endl;
    //sequence solvlets now contain all the solvlets for the entire sequence

    //rearrange this into one vector of solvlets for evaluation
    std::vector<Solvlet> retSolve;
    for (const auto &i : sequenceSolvlets)
      for (const auto &j : i)
        retSolve.push_back(j);

    std::vector<Solvlet> passedSolves;

    const auto acceptLockframeThreshold = params.at(
      COMMON_TLPS_SOLVER_PARAMETERS::LOCKFRAME_THRESHOLD().name());
    if (acceptLockframeThreshold > 0.0f) //if it's greater than zero, we want to evaluate solves before returning them
    {
      for (const auto &solvlet : retSolve)
      {
        const auto thisFrameID = solvlet.getFrameID();
        const auto thisFrame = frames.at(thisFrameID);
        if (evaluateSolution(thisFrame, solvlet.getLabels(), params) >= acceptLockframeThreshold) //if the frame passed the test
        {
          passedSolves.push_back(solvlet);
          //generate lockframe
          auto *lockframe = new Lockframe();
          lockframe->SetImageFromPath(thisFrame->GetImagePath());
          lockframe->SetMaskFromPath(thisFrame->GetMaskPath());
          lockframe->setID(thisFrameID);
          //parent skeleton is the basis, this will copy scale and other params, but have different locations
          lockframe->setSkeleton(solvlet.toSkeleton(thisFrame->getSkeleton()));
          lockframe->setParentFrameID(thisFrameID);

          delete thisFrame; //remove the existing frame
          frames.at(thisFrameID) = lockframe;
        }
      }
    }

    sequence.setFrames(frames);
    for (auto f : frames) //clean up frames
      delete f;
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

    const auto maxFrameHeight = params.at(
      COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().name());

    auto mask = frame->getMask().clone();

    const auto factor = maxFrameHeight / static_cast<float>(mask.rows);
    //compute the scaling factor
    if (spelHelper::compareFloat(factor, 0.0f) > 0 && 
      spelHelper::compareFloat(factor, 1.0f) != 0)
    {
      resize(mask, mask, cvSize(static_cast<int>(mask.cols * factor), 
        static_cast<int>(mask.rows * factor)));
      for (auto &label : labels)
        label.Resize(factor);
    }

    auto correctPixels = 0.0f, incorrectPixels = 0.0f;

    for (auto i = 0; i < mask.cols; ++i) //at every col - x
    {
      for (auto j = 0; j < mask.rows; ++j) //and every row - y
      {
        //check whether pixel hit a label from solution
        auto labelHit = false;
        for (const auto &label : labels)
        {
          if (label.containsPoint(cv::Point2f(static_cast<float>(i), static_cast<float>(j)))) //this is done in x,y coords
          {
            labelHit = true;
            break;
          }
        }
        //check pixel colour
        const auto blackPixel = (mask.at<uchar>(j, i) < 10);

        if (blackPixel && labelHit) //if black in label, incorrect
          ++incorrectPixels;
        else if (!blackPixel && !labelHit) //if white not in label, incorret
          ++incorrectPixels;
        else if (!blackPixel && labelHit)//otherwise correct
          ++correctPixels;
      }
    }

    auto solutionEval = correctPixels / (correctPixels + incorrectPixels);

    //now check for critical part failures - label mostly outside of mask

    std::vector<std::pair<int, float>> badLabelScores;
    const auto badLabelThresh = params.at(
      COMMON_SOLVER_PARAMETERS::BAD_LABEL_THRESH().name());

    for (const auto &label : labels)
    {
      const auto poly = label.getPolygon(); //get the label polygon
      //compute min and max x and y
      //float xMin, xMax, yMin, yMax;
      std::vector<float> xS = { poly[0].x, poly[1].x, poly[2].x, poly[3].x };
      std::vector<float> yS = { poly[0].y, poly[1].y, poly[2].y, poly[3].y };
      const auto xMin = static_cast<int>(*(min_element(xS.begin(), xS.end())));
      const auto xMax = static_cast<int>(*(max_element(xS.begin(), xS.end())));
      const auto yMin = static_cast<int>(*(min_element(yS.begin(), yS.end())));
      const auto yMax = static_cast<int>(*(max_element(yS.begin(), yS.end())));

      auto labelPixels = 0U;
      auto badLabelPixels = 0U;

      for (auto x = xMin; x < xMax; ++x)
      {
        for (auto y = yMin; y < yMax; ++y)
        {
          if (label.containsPoint(cv::Point2f(static_cast<float>(x), static_cast<float>(y))))
          {
            const auto blackPixel = (mask.at<uchar>(y, x) < 10);
            ++labelPixels;
            if (blackPixel)
              ++badLabelPixels;
          }
        }
      }

      const auto labelRatio = 1.0f - static_cast<float>(badLabelPixels) / 
        static_cast<float>(labelPixels); //high is good

      if (labelRatio < badLabelThresh && !label.getIsOccluded()) //not weak, not occluded, badly localised
        badLabelScores.push_back(std::make_pair(label.getLimbID(), labelRatio));
    }

    if (SpelObject::getDebugLevel() >= 1)
    {
      for (const auto &badL : badLabelScores)
      {
        std::stringstream ss;
        ss << "Part " << badL.first << " is badly localised, with score " << badL.second;
        DebugMessage(ss.str(), 1);
      }
    }

    if (badLabelScores.size() != 0) //make the solution eval fail if a part is badly localised
      solutionEval -= 1.0f;

    if (SpelObject::getDebugLevel() >= 1)
    {
      std::stringstream ss;
      ss << "Solution evaluation score - " << solutionEval << " for frame " << frame->getID() << " solve from " << frame->getParentFrameID();
      DebugMessage(ss.str(), 1);
    }
    frame->UnloadAll();

    return solutionEval;
  }

  //compute label score
  float TLPSSolver::computeScoreCost(const LimbLabel& label, std::map<std::string, float> params)
  {
    emplaceDefaultParameters(params);

    const std::string hogName = "18500";
    const std::string csName = "4409412";
    const std::string surfName = "21316";

    //@FIX
    const auto useHoG =
      params.at(COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR().name());
    const auto useCS =
      params.at(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR().name());
    const auto useSURF =
      params.at(COMMON_DETECTOR_PARAMETERS::USE_SURF_DETECTOR().name());

    //TODO: Fix score combinations
    const auto &scores = label.getScores();

    //compute the weighted sum of scores
    auto finalScore = 0.0f;
    auto hogFound = false;
    auto csFound = false;
    auto surfFound = false;
    for (const auto &i : scores)
    {
      auto score = i.getScore();
      if (spelHelper::compareFloat(score, -1.0f) == 0)//if score is -1, set it to 1
        score = 1.0f; //set a high cost for invalid scores
      const auto &name = i.getDetName();
      if (name == hogName)
      {
        finalScore += (score * useHoG);
        hogFound = true;
      }
      else if (name == csName)
      {
        finalScore += (score * useCS);
        csFound = true;
      }
      else if (name == surfName)
      {
        finalScore += (score * useSURF);
        surfFound = true;
      }
    }

    //now add 1.0*coeff for each not found score in this label, that should have been there (i.e., assume the worst)
    finalScore += 1.0f * useHoG * (!hogFound) + 1.0f * useCS * (!csFound) + 1.0f * useSURF *(!surfFound);

    return finalScore;
  }

  //compute distance to parent limb label
  float TLPSSolver::computeJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, bool toChild)
  {
    emplaceDefaultParameters(params);
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
