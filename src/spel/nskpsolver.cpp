// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

// OpenGM
#include <opengm/graphicalmodel/space/discretespace.hxx>
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/operations/minimizer.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>

#include "lockframe.hpp"

#include "colorHistDetector.hpp"
#include "hogDetector.hpp"
#include "surfDetector.hpp"

#include "nskpsolver.hpp"
#include "tlpssolver.hpp"

#include "spelParameters.hpp"


namespace SPEL
{
  NSKPSolver::NSKPSolver() 
  {
    m_id = 1; //this should be unique
    m_name = "NSKP"; //this should be unique
  }

  NSKPSolver::~NSKPSolver()  //inherited virtual
  {

  }

  std::vector<Solvlet> NSKPSolver::solve(Sequence& sequence) 
  {
    std::map<std::string, float> params; //set the default parameters vector
    emplaceDefaultParameters(params);

    //pass to next level solve function, for ISM computing
    return solve(sequence, params);
  }

  std::vector<Solvlet> NSKPSolver::solve(Sequence& sequence, 
    std::map<std::string, float> params) 
  {
    //compute the ISM here
    ImagePixelSimilarityMatrix ISM(sequence.getFrames());
    emplaceDefaultParameters(params);
    //pass to solve function
    return solve(sequence, params, ISM);
  }

  std::vector<Solvlet> NSKPSolver::solve(Sequence& sequence, 
    std::map<std::string, float>  params, 
    const ImageSimilarityMatrix& ism) 
  {
    emplaceDefaultParameters(params);

    const auto nskpIters = static_cast<uint32_t>(params.at(
      COMMON_NSKP_SOLVER_PARAMETERS::ITERATIONS().name()));

    std::vector<Solvlet> solvlets;
    sequence.computeInterpolation(params); //interpolate the sequence first
    //propagate keyframes
    auto propagatedFrames = sequence.getFrames();

    auto lockframesLastIter = 0U;
    std::vector<int> ignore; //frames to ignore during propagation

    const auto trees = buildFrameMSTs(ism, params);

    for (auto iteration = 0U; iteration < nskpIters; ++iteration)
    {
      const auto sol = propagateKeyframes(propagatedFrames, params, ism, 
        trees, ignore);

      //add the new solves to the return vector
      for (const auto &s : sol)
        solvlets.push_back(s);

      //calculate number of lockframes in the sequence
      auto numLockframes = 0U;
      for (const auto frameIter : propagatedFrames)
        if (frameIter->getFrametype() == LOCKFRAME)
          numLockframes++;
      //terminate loop if no more lockframes are generated
      if (numLockframes == lockframesLastIter) 
      {
        std::stringstream ss;
        ss << "Terminating keyframe propagation after " << iteration << 
          " iterations.";
        DebugMessage(ss.str(), 5);
        break;
      }
      lockframesLastIter = numLockframes;
    }
    //progressFunc(1.0);
    
    sequence.setFrames(propagatedFrames); //set the new frames to sequence
    //delete the frame vector as it is no longer being used
    for (auto p : propagatedFrames) 
      delete p;

    if (spelHelper::compareFloat(params.at(
      COMMON_NSKP_SOLVER_PARAMETERS::USE_TLPS().name()), 0.0f) > 0)
    {
      TLPSSolver tlps;
      return tlps.solve(sequence, params, solvlets);
    }
    sort(solvlets.begin(), solvlets.end());
    return solvlets;
  }

  std::vector<NSKPSolver::SolvletScore> NSKPSolver::propagateFrame(
    const int frameId, const std::vector<Frame*> &frames, 
    std::map<std::string, float> params, const ImageSimilarityMatrix& ism, 
    const std::vector<MinSpanningTree>& trees, 
    std::vector<int>& ignore) const 
  {
    if (frames.size() == 0)
    {
      const std::string str = "No frames found.";
      DebugMessage(str, 1);
      throw std::out_of_range(str);
    }

    emplaceDefaultParameters(params);

    params.at(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().name()) =
      std::min(params.at(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().name()),
        static_cast<float>(frames.front()->getFrameSize().height));

    std::vector<NSKPSolver::SolvletScore> allSolves;
    const auto imageHeight = 
      params.at(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().name());

    //detector search parameters
    const auto baseRotationRange = 
      params.at(COMMON_SOLVER_PARAMETERS::BASE_ROTATION_RANGE().name());
    params.at(DETECTOR_DETECT_PARAMETERS::STEP_THETA().name()) =
      baseRotationRange / 4.0f;
    params.at(COMMON_SOLVER_PARAMETERS::BASE_SEARCH_RADIUS().name()) =
      imageHeight / 30.0f;
    const auto baseSearchRadius = static_cast<uint32_t>(params.at(
      COMMON_SOLVER_PARAMETERS::BASE_SEARCH_RADIUS().name()));
    params.at(COMMON_SOLVER_PARAMETERS::BASE_ROTATION_STEP().name()) =
      baseSearchRadius / 10.0f;

    const auto depthRotationCoeff = 
      params.at(
        COMMON_SOLVER_PARAMETERS::PART_DEPTH_ROTATION_COEFFICIENT().name());

    const auto baseRotationStep = 
      params.at(COMMON_SOLVER_PARAMETERS::BASE_ROTATION_STEP().name());
    const auto baseSearchStep = 
      params.at(COMMON_SOLVER_PARAMETERS::BASE_SEARCH_STEP().name());

    const auto useHoG = spelHelper::compareFloat(params.at(
      COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR().name()), 0.0f) > 0;
    const auto useCS = spelHelper::compareFloat(params.at
    (COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR().name()), 0.0f) > 0;
    const auto useSURF = spelHelper::compareFloat(params.at(
      COMMON_DETECTOR_PARAMETERS::USE_SURF_DETECTOR().name()), 0.0f) > 0;

    const auto propagateFromLockframes = 
      spelHelper::compareFloat(params.at(
        COMMON_NSKP_SOLVER_PARAMETERS::PROPAGATE_FROM_LOCKFRAMES().name()), 
        1.0f) == 0;

    const auto currentFrame = frames.at(frameId);
    const auto currentFrameId = currentFrame->getID();
    const auto isIgnored = 
      std::binary_search(ignore.begin(), ignore.end(), currentFrameId);
    //get the MST, by ID, as in ISM
    const auto mst = trees.at(currentFrameId).getMST(); 
    const auto currentFrameType = currentFrame->getFrametype();
    //as long as it's not an interpolated frame, and not on the ignore list
    //and, if it's a lockframe, and solving from lockframes is allowed
    if (currentFrameType != INTERPOLATIONFRAME && !isIgnored 
      && (currentFrameType != LOCKFRAME || propagateFromLockframes) && 
      mst.size() > 1) 
    {
      std::vector<Detector*> detectors;
      if (useCS)
        detectors.push_back(new ColorHistDetector());
      if (useHoG)
        detectors.push_back(new HogDetector());
      if (useSURF)
        detectors.push_back(new SurfDetector());

      std::vector<Frame*> trainingFrames(1, currentFrame);

      for (auto &detector : detectors)
        detector->train(trainingFrames, params);

      //@TODO: This may need to be modified to propagate not from root frame, 
      //but from parent frame
      //but only if the solve was successful, 
      //otherwise propagate from the root frame
      for (auto mstIter = mst.begin(); mstIter != mst.end(); ++mstIter) 
      {
        ///define the space
        typedef opengm::DiscreteSpace<> Space;
        ///define the model
        typedef opengm::GraphicalModel<float, opengm::Adder, 
          opengm::ExplicitFunction<float>, Space> Model;
        ///define the update rules
        typedef opengm::BeliefPropagationUpdateRules<Model, 
          opengm::Minimizer> UpdateRules;
        ///define the inference algorithm
        typedef opengm::MessagePassing<Model, opengm::Minimizer, UpdateRules, 
          opengm::MaxDistance> BeliefPropagation;

        const auto mstIterFrame = frames.at(*mstIter);
        const auto mstIterFrameType = mstIterFrame->getFrametype();
        //don't push to existing keyframes and lockframes
        //also ignore mst frame if it's this frame
        if (mstIterFrameType == KEYFRAME || mstIterFrameType == LOCKFRAME || 
          *mstIter == frameId) 
          continue; 

        //check whether parent is a lockframe
        auto parentIsLockframe = false;
        SolvletScore *parentIsSolved = nullptr;
        const auto parentFrameIdx = *mst.parent(mstIter);
        auto parentFrame = frames.at(parentFrameIdx);
        if (mstIter != mst.begin())
        {
          const auto parentFrameType = parentFrame->getFrametype();

          if (parentFrameType == LOCKFRAME || parentFrameType == KEYFRAME)
            parentIsLockframe = true;
          for (const auto &i : allSolves) //check among accepted solutions
            if (i.solvlet.getFrameID() == parentFrameIdx)
              parentIsSolved = const_cast<SolvletScore*>(&i);
        }

        auto lockframe = new Lockframe();

        //if solved, set the prior to the solve
        if (parentIsSolved != nullptr) 
          lockframe->setSkeleton(parentIsSolved->solvlet.toSkeleton(
            currentFrame->getSkeleton()));
        //if the parent of this node is a lockframe, use it as a prior
        else if (parentIsLockframe) 
          lockframe->setSkeleton(parentFrame->getSkeleton());
        //otherwise use the root frame as a prior
        else 
          lockframe->setSkeleton(currentFrame->getSkeleton());

        const auto mstIterFrameId = mstIterFrame->getID();
        lockframe->setID(mstIterFrameId);
        lockframe->SetImageFromPath(mstIterFrame->GetImagePath());
        lockframe->SetMaskFromPath(mstIterFrame->GetMaskPath());

        auto skeleton = lockframe->getSkeleton();

        const auto &partTree = skeleton.getPartTree();

        for (auto partIter = partTree.begin(); partIter != partTree.end(); 
          ++partIter)
        {
          auto rotationRange = baseRotationRange;
          auto searchRange = baseSearchRadius * pow(depthRotationCoeff, 
            partTree.depth(partIter));

          if (partTree.number_of_children(partIter) == 0)
          {
            searchRange = searchRange * 2;
            rotationRange = rotationRange * depthRotationCoeff;
          }

          partIter->setRotationSearchRange(rotationRange);
          partIter->setSearchRadius(searchRange);
        }
        skeleton.setPartTree(partTree);
        lockframe->setSkeleton(skeleton);

        if (parentIsLockframe)
          parentFrame->setSkeleton(skeleton);
        else
          currentFrame->setSkeleton(skeleton);

        //compute the shift between the frame we are propagating from 
        //and the current frame
        auto parentFrameId = parentFrame->getID();
        cv::Point2f shift;
        if (parentIsSolved != nullptr)
          shift = ism.getShift(parentIsSolved->solvlet.getFrameID(), 
            mstIterFrameId);
        else if (parentIsLockframe)
          shift = ism.getShift(parentFrameId, mstIterFrameId);
        else
          shift = ism.getShift(currentFrameId, mstIterFrameId);
        //shift the skeleton by the correct amount
        lockframe->shiftSkeleton2D(shift); 

        std::map<uint32_t, std::vector<LimbLabel> > labels;
        for (const auto detector : detectors)
          labels = detector->detect(lockframe, params, labels);

        const auto maxPartCandidates = static_cast<int>(params.at(
          COMMON_SOLVER_PARAMETERS::MAX_PART_CANDIDATES().name()));

        for (auto &label : labels) //for each part
        {
          std::vector<LimbLabel> tmp;
          const auto limit = static_cast<uint32_t>(label.second.size() < maxPartCandidates ? 
            label.second.size() : maxPartCandidates);
          for (auto j = 0U; j < limit; ++j)
            tmp.push_back(label.second.at(j));
          //set this part's candidates to the new trimmed vector
          label.second = tmp;
        }

        std::vector<size_t> numbersOfLabels; //numbers of labels per part

        for (const auto &label : labels)
          //numbers of labels now contains the numbers
          numbersOfLabels.push_back(label.second.size()); 

        Space space(numbersOfLabels.begin(), numbersOfLabels.end());
        Model gm(space);

        auto jointFactors = 0U;
        //label score cost
        for (auto partIter = partTree.begin(); partIter != partTree.end(); 
          ++partIter)
        {
          const auto currentPartId = partIter->getPartID();
          const auto &currentLabel = labels.at(currentPartId);
          //create vector of indices of variables
          std::vector<int> varIndices (1, currentPartId); 
          //number of labels
          size_t scoreCostShape[] = { numbersOfLabels.at(currentPartId) }; 
          opengm::ExplicitFunction<float> scoreCostFunc(scoreCostShape, 
            scoreCostShape + 1); //explicit function declare

          auto i = 0;
          for (const auto &c : currentLabel)
            scoreCostFunc(i++) = computeScoreCost(c, params);

          gm.addFactor(gm.addFunction(scoreCostFunc), varIndices.begin(), 
            varIndices.end()); //bind to factor and variables
          //if iterator is not on root node, there is always a parent body part
          if (partIter != partTree.begin()) 
          {
            varIndices.clear();
            //find the parent of this part
            auto parentPartIter = partTree.parent(partIter); 
            const auto parentPartId = parentPartIter->getPartID();
            const auto &parentLabel = labels.at(parentPartId);
            //push back parent partID as the second variable index
            varIndices.push_back(parentPartId); 
            //push first value in (parent, this)
            varIndices.push_back(currentPartId); 

            size_t jointCostShape[] = { numbersOfLabels.at(parentPartId), 
              numbersOfLabels.at(currentPartId) }; //number of labels
            opengm::ExplicitFunction<float> jointCostFunc(jointCostShape, 
              jointCostShape + 2); //explicit function declare

            //first figure out which of the current body part's joints 
            //should be in common with the parent body part
            auto toChild = parentPartIter->getChildJoint() == 
              partIter->getParentJoint() ? true : false;

            auto jointMax = 0.0f;
            for (const auto &c : currentLabel)
            {
              for (const auto &p: parentLabel)
              {
                const auto val = computeJointCost(c, p, toChild);

                if (val > jointMax && val != std::numeric_limits<float>::max())
                  jointMax = val;
              }
            }

            for (auto c = 0U; c < currentLabel.size(); ++c)
              for (auto p = 0U; p < parentLabel.size(); ++p)
                jointCostFunc(p, c) = computeNormJointCost(currentLabel.at(c), 
                  parentLabel.at(p), params, jointMax, toChild);

            //explicit function add to graphical model
            const auto jointFid = gm.addFunction(jointCostFunc); 
            //bind to factor and variables
            gm.addFactor(jointFid, varIndices.begin(), varIndices.end()); 
            jointFactors++;
          }
        }

        BeliefPropagation bp(gm, 
          BeliefPropagation::Parameter (100, 1e-7f, 0.5f));
        bp.infer();

        const auto labelsCount = labels.size();
        // obtain the (approximate) argmin
        std::vector<size_t> labeling(labelsCount);
        bp.arg(labeling);

        std::vector<LimbLabel> solutionLabels;
        solutionLabels.reserve(labelsCount);
        for (auto i = 0; i < labelsCount; ++i)
          //pupulate solution vector
          solutionLabels.push_back(labels.at(i).at(labeling.at(i))); 

        //labeling now contains the approximately optimal labels 
        //for this problem
        Solvlet solvlet(*mstIter, solutionLabels);
        SolvletScore ss;
        ss.solvlet = solvlet;
        if (parentIsSolved != nullptr)
          ss.parentFrame = parentIsSolved->solvlet.getFrameID();
        else if (parentIsLockframe)
          ss.parentFrame = frames.at(parentFrameIdx)->getID();
        else
          ss.parentFrame = currentFrameId;
        ss.score = evaluateSolution(frames.at(solvlet.getFrameID()),
          solvlet.getLabels(), params);
        allSolves.push_back(ss);

        delete lockframe; //delete the unused pointer now
      }

      //do detector cleanup
      for (auto detector : detectors)
        delete detector;
      detectors.clear();
      //add this frame to the ignore list for future iteration, 
      //so that we don't propagate from it twice
      ignore.push_back(currentFrameId); 
    }
    
    return allSolves;
  }

  void NSKPSolver::emplaceDefaultParameters(
    std::map<std::string, float>& params) const 
  {
    Solver::emplaceDefaultParameters(params);
    spelHelper::mergeParameters(params, 
      COMMON_NSKP_SOLVER_PARAMETERS::getParameters());
  }

  std::vector<Solvlet> NSKPSolver::propagateKeyframes(
    const std::vector<Frame*>& frames, std::map<std::string, float> params, 
    const ImageSimilarityMatrix& ism, 
    const std::vector<MinSpanningTree>& trees, 
    std::vector<int>& ignore) const 
  {
    emplaceDefaultParameters(params);

    if (frames.size() == 0)
      return std::vector<Solvlet>();

    std::vector<std::vector<SolvletScore> > allSolves;
    for (auto frameId = 0U; frameId < frames.size(); ++frameId)
      allSolves.push_back(propagateFrame(frameId, frames, params, ism, trees, 
        ignore));

    //now extract the best solves
    auto acceptLockframeThreshold = params.at(
      COMMON_NSKP_SOLVER_PARAMETERS::LOCKFRAME_THRESHOLD().name());

    std::map<uint32_t, SolvletScore> bestSolves;
    for (const auto &frameSolves : allSolves)
    {
      for (const auto &solve : frameSolves)
      {
        if (solve.score >= acceptLockframeThreshold)
        {
          const auto id = solve.solvlet.getFrameID();
          if (bestSolves.find(id) == bestSolves.cend())
            //emplace this solve if one isn't in there yet
            bestSolves.emplace(id, solve); 
          //now compare to solve in the map
          else if (solve.score > bestSolves.at(id).score) 
            //replace with this one if it's better
            bestSolves.at(id) = solve; 
        }
      }
    }

    //now create skeletons for these solves
    std::vector<Lockframe*> lockframes;
    for (const auto &bs : bestSolves)
    {
      const auto &ss = bs.second;
      const auto thisFrameID = ss.solvlet.getFrameID();
      
      auto lockframe = new Lockframe();
      const auto currentFrame = frames.at(thisFrameID);
      lockframe->SetImageFromPath(currentFrame->GetImagePath());
      lockframe->SetMaskFromPath(currentFrame->GetMaskPath());
      lockframe->setID(thisFrameID);
      //parent skeleton is the basis, this will copy scale and other params, 
      //but have different locations
      lockframe->setSkeleton(ss.solvlet.toSkeleton(
        currentFrame->getSkeleton()));
      lockframe->setParentFrameID(ss.parentFrame);

      lockframes.push_back(lockframe);
    }

    std::vector<Solvlet> solvlets;

    for (auto lockframe : lockframes)
    {
      const auto lockframeId = lockframe->getID();
      Frame *frame = nullptr;
      for (auto f : frames)
      {
        if (f->getID() == lockframeId)
        {
          frame = f;
          break;
        }
      }
      const auto frameType = frame->getFrametype();
      //never replace keyframes and existing lockframes
      if (frameType != LOCKFRAME && frameType != KEYFRAME) 
      {
        //delete the frame currently there, and replace with lockframe
        delete frame; 
        //make pointers point to the correct objects
        *frame = *lockframe; 
        solvlets.push_back(bestSolves.at(lockframeId).solvlet);
      }
      else
        delete lockframe;
    }

    if (SpelObject::getDebugLevel() >= 5)
    {
      std::stringstream ss;
      ss << "Generated " << lockframes.size() << " lockframes!";
      DebugMessage(ss.str(), 5);      
    }

    return solvlets;
  }

  //compute label score
  float NSKPSolver::computeScoreCost(const LimbLabel& label, 
    std::map<std::string, float> params) const 
  {
    emplaceDefaultParameters(params);

    auto sumcoeff = 
      params.at(COMMON_DETECTOR_PARAMETERS::USE_HOG_DETECTOR().name()) + 
      params.at(COMMON_DETECTOR_PARAMETERS::USE_CH_DETECTOR().name()) +
      params.at(COMMON_DETECTOR_PARAMETERS::USE_SURF_DETECTOR().name());

    const auto scores = label.getScores();

    //compute the weighted sum of scores
    auto finalScore = 0.0f;
    std::set<std::string> usedCoeffs;
    for (const auto &s : scores)
    {
      auto score = s.getScore();
      if (spelHelper::compareFloat(score, -1.0f) == 0)
        score = 1.0f;
      const auto detector = s.getDetName();
      const auto coeff = s.getCoeff();
      if (usedCoeffs.count(detector) == 0)
      {
        usedCoeffs.insert(detector);
        sumcoeff -= coeff;
      }
      finalScore += (score * coeff);
    }
    if (spelHelper::compareFloat(sumcoeff, 0.0f) > 0)
      finalScore += sumcoeff;

    return finalScore;
  }

  //compute distance to parent limb label
  float NSKPSolver::computeJointCost(const LimbLabel& child, 
    const LimbLabel& parent, bool toChild) const 
  {
    cv::Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    child.getEndpoints(c0, c1);
    parent.getEndpoints(p0, p1);

    //normalise this?

    //return the squared distance from the lower parent joint p1, 
    //to the upper child joint c0
    if (toChild)
      return sqrt(pow((c0.x - p1.x), 2) + pow((c0.y - p1.y), 2));
    else //otherwise we're connected to the parent's parent body part
      return sqrt(pow((c0.x - p0.x), 2) + pow((c0.y - p0.y), 2));
  }

  //compute distance to parent limb label
  float NSKPSolver::computeNormJointCost(const LimbLabel& child, 
    const LimbLabel& parent, std::map<std::string, float> params, 
    float max, bool toChild) const 
  {
    emplaceDefaultParameters(params);

    //output a sentence about who connected to whom and what the score was
    if (SpelObject::getDebugLevel() >= 1 && (child.getLimbID() == 7 || 
      child.getLimbID() == 6) && !toChild)
    {
      std::stringstream ss;
      ss << "Part " << child.getLimbID() << " is connecting to part " << 
        parent.getLimbID() << " PARENT joint";
      DebugMessage(ss.str(), 1);
    }

    const auto lambda = params.at(
      COMMON_SOLVER_PARAMETERS::IMAGE_COEFFICIENT().name());
    return computeJointCost(child, parent, toChild) / max * lambda;
  }

  //compute distance to the body part prior
  float NSKPSolver::computePriorCost(const LimbLabel& label, 
    const BodyPart& prior, const Skeleton& skeleton) const 
  {
    cv::Point2f p0, p1;
    label.getEndpoints(p0, p1);
    const auto pp0 = 
      skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
    const auto pp1 = 
      skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

    //normalise this cost?

    //return the sum of squared distances between the corresponding joints, 
    //prior to label
    return pow((p0.x - pp0.x), 2) + pow((p0.y - pp0.y), 2) + 
      pow((p1.x - pp1.x), 2) + pow((p1.y - pp1.y), 2);
  }

  float NSKPSolver::computeNormPriorCost(const LimbLabel& label, 
    const BodyPart& prior, const Skeleton& skeleton, 
    std::map<std::string, float> params, float max) const 
  {
    emplaceDefaultParameters(params);

    const auto lambda = 
      params.at(COMMON_SOLVER_PARAMETERS::PRIOR_COEFFICIENT().name());

    //return the sum of squared distances between the corresponding joints, 
    //prior to label
    return lambda * computePriorCost(label, prior, skeleton) / max;
  }

  //build an MST for every frame and return the vector
  std::vector<MinSpanningTree > NSKPSolver::buildFrameMSTs(
    const ImageSimilarityMatrix &ism, 
    std::map<std::string, float> params) const 
  {
    const auto simThresh = 1.0f + 3.5f * ism.stddev() / ism.min();

    const auto treeSize = ism.size();

    std::vector<MinSpanningTree> frameMST;
    frameMST.reserve(treeSize);
    //vector<vector<int> > frameMSTvec;

    for (auto i = 0U; i < treeSize; ++i)
    {
      //for each frame, build an MST
      //and add to vector
      frameMST.push_back(MinSpanningTree(ism, i, treeSize, simThresh));
      if (SpelObject::getDebugLevel() >= 5)
      {
        std::stringstream ss;
        ss << i << " MST built";
        DebugMessage(ss.str(), 5);
      }
    }

    return frameMST;
  }

  //suggest maximum number of keyframes
  //function should return vector with suggested keyframe numbers
  std::vector<std::pair<int, int>> NSKPSolver::suggestKeyframes(
    const ImageSimilarityMatrix& ism, 
    std::map<std::string, float> params) const 
  {
    emplaceDefaultParameters(params);

    if (SpelObject::getDebugLevel() >= 5)
    { 
      const std::string s = "Building all MSTs...";
      DebugMessage(s, 5);
    }

    const auto mstVec = buildFrameMSTs(ism, params);

    if (SpelObject::getDebugLevel() >= 5)
    {
      const std::string s = "Finished building MSTs";
      DebugMessage(s, 5);
    }

    auto minKeyframeDist = static_cast<int32_t>(params.at(
      COMMON_NSKP_SOLVER_PARAMETERS::MIN_KEY_FRAME_DISTANCE().name()));
    std::vector<std::vector<uint32_t> > orderedList;
    for (const auto &i : mstVec)
    {
      const auto MST = i.getMST();
      std::vector<uint32_t> frames;
      frames.reserve(MST.size());
      std::copy(MST.begin(), MST.end(), frames.begin());
      orderedList.push_back(frames);
    }
    //this is the simple way of counting the number of frames that made it in
    //alternatively we could come up with a more complex scheme 
    //for determining keyframe optimality
    std::vector<std::pair<int, int>> frameOrder;
    frameOrder.reserve(orderedList.size());
    std::sort(orderedList.begin(), orderedList.end(), 
      [&](const auto &one, const auto &two)
    {
      return one.size() < two.size();
    });
    while (orderedList.size() != 0)
    {
      const auto curItem = orderedList.back();
      const auto curItemSize = curItem.size();
      //if there largest vector remaining is of legnth zero, stop
      if (curItemSize <= 1)
        break;

      frameOrder.push_back(std::make_pair(curItem.front(), 
        static_cast<int>(curItemSize)));

      //remove all values in that vector from all others
      for (auto &i : orderedList)
        //for each element in erasedVector
        for (const auto &j : curItem)
          i.erase(std::remove(i.begin(), i.end(), j), i.end());

      orderedList.pop_back();
    }
    //now tidy up the frame order by forcing minimum keyframe distance
    std::vector<std::pair<int, int>> aux;
    aux.push_back(frameOrder.front());
    for (const auto &i : frameOrder)
    {
      auto isOk = true;
      const auto thisFrame = i.first;
      for (const auto &j : aux)
      {
        const auto thatFrame = j.first;
        if (std::abs(thisFrame - thatFrame) < minKeyframeDist)
        {
          isOk = false;
          break;
        }
      }

      if (isOk) //if it's ok so far, add it to final
        aux.push_back(i);
    }
    //return the resulting vector
    
    return aux;
  }

  float NSKPSolver::evaluateSolution(Frame* frame, 
    std::vector<LimbLabel> labels, 
    std::map<std::string, float> params) const 
  {
    emplaceDefaultParameters(params);

    // /*
    //   There should clearly be several factors that affect the outcome 
    //   of an evaluation:
    //   1) Mask coverage
    //   2) Parts falling outside mask range
    //   3) ?
    //   */

    //engaged pixles - the total number of pixels that are either within 
    //limb label of within the mask
    //correct pixels - those pixles that are black and outside a label, 
    //and white and inside a label
    //incorrect pixels - those pixels that are black and inside a label, 
    //and white and outside a label
    //score = correct/(correct+incorrect)

    const auto maxFrameHeight = static_cast<uint32_t>(
      params.at(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().name()));

    auto mask = frame->getMask().clone();

    //compute the scaling factor
    if (maxFrameHeight != 0)
    {
      const auto factor = 
        static_cast<float>(maxFrameHeight) / static_cast<float>(mask.rows);
      resize(mask, mask, cvSize(static_cast<int>(mask.cols * factor), 
        static_cast<int>(mask.rows * factor)));
      for (auto &label : labels)
        label.Resize(factor);
    }

    auto correctPixels = 0U, incorrectPixels = 0U;

    for (auto i = 0; i < mask.cols; ++i) //at every col - x
    {
      for (auto j = 0; j < mask.rows; ++j) //and every row - y
      {
        //check whether pixel hit a label from solution
        auto labelHit = false;
        const auto point = 
          cv::Point2f(static_cast<float>(i), static_cast<float>(j));
        for (const auto &label : labels)
        {
          if (label.containsPoint(point)) //this is done in x,y coords
          {
            labelHit = true;
            break;
          }
        }

        //check pixel colour
        const auto intensity = mask.at<uchar>(j, i); //this is done with reve
        const auto blackPixel = (intensity < 10);

        if (blackPixel && labelHit) //if black in label, incorrect
          incorrectPixels++;
        else if (!blackPixel)
        {
          if (!labelHit) //if white not in label, incorret
            incorrectPixels++;
          else if (labelHit)//otherwise correct
            correctPixels++;
        }
      }
    }

    //now check for critical part failures - label mostly outside of mask

    auto badLabelScore = 0U;
    const auto badLabelThresh = 
      params.at(COMMON_SOLVER_PARAMETERS::BAD_LABEL_THRESH().name());

    for (const auto &label : labels)
    {
      const auto poly = label.getPolygon(); //get the label polygon
      const std::vector<float> xS = 
      { poly[0].x, poly[1].x, poly[2].x, poly[3].x };
      const std::vector<float> yS = 
      { poly[0].y, poly[1].y, poly[2].y, poly[3].y };
      const auto xMin = min_element(xS.begin(), xS.end());
      const auto xMax = max_element(xS.begin(), xS.end());
      const auto yMin = min_element(yS.begin(), yS.end());
      const auto yMax = max_element(yS.begin(), yS.end());

      auto labelPixels = 0;
      auto badLabelPixels = 0;

      for (auto x = *xMin; x < *xMax; ++x)
      {
        for (auto y = *yMin; y < *yMax; ++y)
        {
          if (label.containsPoint(cv::Point2f(x, y)))
          {
             //this is done with reverse y,x
            const auto intensity = 
              mask.at<uchar>(static_cast<int>(y), static_cast<int>(x));
            labelPixels++;
            if (intensity < 10)
              ++badLabelPixels;
          }
        }
      }

      const auto labelRatio = 1.0f - static_cast<float>(badLabelPixels) / 
        static_cast<float>(labelPixels); //high is good
      //not weak, not occluded, badly localised
      if (labelRatio < badLabelThresh && !label.getIsOccluded())
        ++badLabelScore;
    }

    auto solutionEval = static_cast<float>(correctPixels) / 
      static_cast<float>(correctPixels + incorrectPixels);
    //make the solution eval fail if a part is badly localised
    if (badLabelScore != 0) 
      solutionEval -= 1.0f;

    if (SpelObject::getDebugLevel() >= 1)
    {
      std::stringstream ss;
      ss << "Solution evaluation score - " << solutionEval << 
        " for frame " << frame->getID() << " solve from " << 
        frame->getParentFrameID();
      DebugMessage(ss.str(), 1);
    }
    frame->UnloadAll();

    return solutionEval;
  }

}
