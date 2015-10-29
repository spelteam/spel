#include "nskpsolver.hpp"

namespace SPEL
{
  NSKPSolver::NSKPSolver()
  {
    id = 1; //this should be unique
    name = "NSKP"; //this should be unique
  }

  NSKPSolver::~NSKPSolver() //inherited virtual
  {

  }

  std::vector<Solvlet> NSKPSolver::solve(Sequence& sequence) //inherited virtual
  {
    std::map<std::string, float> params; //set the default parameters vector

    //set some parameter defaults

    //pass to next level solve function, for ISM computing
    return this->solve(sequence, params);
  }

  std::vector<Solvlet> NSKPSolver::solve(Sequence& sequence, std::map<std::string, float> params) //inherited virtual
  {
    //compute the ISM here
    ImageSimilarityMatrix ISM(sequence.getFrames());

    //pass to solve function
    return this->solve(sequence, params, ISM);
  }

  std::vector<Solvlet> NSKPSolver::solve(Sequence& sequence, std::map<std::string, float>  params, const ImageSimilarityMatrix& ism) //inherited virtual
  {
    //parametrise the number of times frames get propagated
    params.emplace("nskpIters", 0); //set number of iterations, 0 to iterate until no new lockframes are introduced

    uint32_t nskpIters = params.at("nskpIters");
    if (nskpIters == 0)
      nskpIters = INT32_MAX;

    std::vector<Solvlet> solvlets;
    sequence.computeInterpolation(params); //interpolate the sequence first
    //propagate keyframes
    std::vector<Frame*> propagatedFrames = sequence.getFrames();

    uint32_t lockframesLastIter = 0;
    std::vector<int> ignore; //frames to ignore during propagation

    std::vector<MinSpanningTree> trees = buildFrameMSTs(ism, params);

    //progressFunc(0.0);
    for (uint32_t iteration = 0; iteration < nskpIters; ++iteration)
    {
      if (iteration >= nskpIters)
        break;

      std::vector<Solvlet> sol = propagateKeyframes(propagatedFrames, params, ism, trees, ignore);

      //add the new solves to the return vector
      for (auto s : sol)
        solvlets.push_back(s);

      //calculate number of lockframes in the sequence
      uint32_t numLockframes = 0;
      for (auto frameIter : propagatedFrames)
        if (frameIter->getFrametype() == LOCKFRAME)
          numLockframes++;

      if (numLockframes == lockframesLastIter) //terminate loop if no more lockframes are generated
      {
        std::cerr << "Terminating keyframe propagation after " << iteration << " iterations." << std::endl;
        break;
      }
      lockframesLastIter = numLockframes;
    }
    //progressFunc(1.0);
    
    sequence.setFrames(propagatedFrames); //set the new frames to sequence

    for (auto p : propagatedFrames) //delete the frame vector as it is no longer being used
      delete p;

    if (params.at("withTLPS"))
    {
      //create tlps solver
      TLPSSolver tlps;
      //return the TLPS solve
      return tlps.solve(sequence, params, solvlets);
    }
    //the params map should countain all necessary parameters for solving, if they don't exist, default values should be used

    sort(solvlets.begin(), solvlets.end());
    return solvlets;
  }

  std::vector<NSKPSolver::SolvletScore> NSKPSolver::propagateFrame(int frameId, const std::vector<Frame*> frames, std::map<std::string, float> params, const ImageSimilarityMatrix& ism, const std::vector<MinSpanningTree>& trees, std::vector<int>& ignore)
  {
    std::vector<NSKPSolver::SolvletScore> allSolves;
    cv::Mat image = frames[0]->getImage();
    // //@Q should frame ordering matter? in this function it should not matter, so no checks are necessary
    // float mst_thresm_multiplier=params.at("mst_thresh_multiplier"); //@FIXME PARAM this is a param, not static
    // int mst_max_size=params.at("mst_max_size"); //@FIXME PARAM this is a param, not static

    //the params vector should contain all necessary parameters, if a parameter is not present, default values should be used
    params.emplace("debugLevel", 1); //set up the lockframe accept threshold by mask coverage
    params.emplace("propagateFromLockframes", 1); //don't propagate from lockframes, only from keyframes

    //detector enablers
    params.emplace("useCSdet", 1.0); //determine if ColHist detector is used and with what coefficient
    params.emplace("useHoGdet", 0.0); //determine if HoG descriptor is used and with what coefficient
    params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient
    params.emplace("maxPartCandidates", 40); //set the max number of part candidates to allow into the solver

    //detector search parameters

    params.emplace("partDepthRotationCoeff", 1.2); // 20% increase at each depth level
    params.emplace("anchorBindDistance", 0); //restrict search regions if within bind distance of existing keyframe or lockframe (like a temporal link
    params.emplace("anchorBindCoeff", 0.0); //multiplier for narrowing the search range if close to an anchor (lockframe/keyframe)
    params.emplace("bindToLockframes", 0); //should binds be also used on lockframes?

    //detector search parameters
    params.emplace("baseRotationRange", 40); //search angle range of +/- 40 degrees
    float baseRotationRange = params.at("baseRotationRange");
    params.emplace("baseRotationStep", baseRotationRange / 4.0); //search with angle step of 10 degrees, this a per-part range and overrides globals
    params.emplace("stepTheta", baseRotationRange / 4.0); //search in a grid every 10 pixels

    params.emplace("baseSearchRadius", image.rows / 30.0); //search a radius of 100 pixels
    int baseSearchRadius = params.at("baseSearchRadius");
    params.emplace("baseSearchStep", baseSearchRadius / 10.0); //do 9-10 steps in each direction
    //solver sensitivity parameters
    params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
    params.emplace("jointCoeff", 0.5); //set solver body part connectivity sensitivitymaxPartCandidates
    params.emplace("jointLeeway", 0.05); //set solver lenience for body part disconnectedness, as a percentage of part length
    params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity

    //solver eval parameters
    params.emplace("nskpLockframeThreshold", 0.52); //set up the lockframe accept threshold by mask coverage

    //restrict problem size
    params.emplace("maxFrameHeight", 288);  //emplace if not defined


    float depthRotationCoeff = params.at("partDepthRotationCoeff");

    float baseRotationStep = params.at("baseRotationStep");
    float baseSearchStep = params.at("baseSearchStep");

    //    //paramaters for soft binding to existing keyframes/lockframes
    //    int anchorBindDistance = params.at("anchorBindDistance");
    //    float anchorBindCoeff = params.at("anchorBindCoeff");
    //    bool bindToLockframes = params.at("bindToLockframes");

    float useHoG = params.at("useHoGdet");
    float useCS = params.at("useCSdet");
    float useSURF = params.at("useSURFdet");
    uint32_t debugLevel = params.at("debugLevel");
    bool propagateFromLockframes = params.at("propagateFromLockframes");

    bool isIgnored = false;
    for (uint32_t i = 0; i < ignore.size(); ++i)
    {
      if (ignore[i] == frames[frameId]->getID())
      {
        isIgnored = true;
        break;
      }
    }

    tree<int> mst = trees[frames[frameId]->getID()].getMST(); //get the MST, by ID, as in ISM
    if (frames[frameId]->getFrametype() != INTERPOLATIONFRAME && !isIgnored //as long as it's not an interpolated frame, and not on the ignore list
      && (frames[frameId]->getFrametype() != LOCKFRAME || propagateFromLockframes) && mst.size()>1) //and, if it's a lockframe, and solving from lockframes is allowed
    {

      tree<int>::iterator mstIter;
      //do OpenGM solve for single factor graph

      std::vector<Detector*> detectors;
      if (useCS)
        detectors.push_back(new ColorHistDetector());
      if (useHoG)
        detectors.push_back(new HogDetector());
      if (useSURF)
        detectors.push_back(new SurfDetector());

      std::vector<Frame*> trainingFrames;
      trainingFrames.push_back(frames[frameId]); //set training frame by index
      
      for (uint32_t i = 0; i < detectors.size(); ++i)
      {
        detectors[i]->setDebugLevel(0);
        detectors[i]->train(trainingFrames, params);
      }

      //@TODO: This may need to be modified to propagate not from root frame, but from parent frame
      //but only if the solve was successful, otherwise propagate from the root frame
      for (mstIter = mst.begin(); mstIter != mst.end(); ++mstIter) //for each frame in the MST
      {
        ///define the space
        typedef opengm::DiscreteSpace<> Space;
        ///define the model
        typedef opengm::GraphicalModel<float, opengm::Adder, opengm::ExplicitFunction<float>, Space> Model;

        ///define the update rules
        typedef opengm::BeliefPropagationUpdateRules<Model, opengm::Minimizer> UpdateRules;
        ///define the inference algorithm
        typedef opengm::MessagePassing<Model, opengm::Minimizer, UpdateRules, opengm::MaxDistance> BeliefPropagation;

        //t1 = high_resolution_clock::now();

        if (frames[*mstIter]->getFrametype() == KEYFRAME || frames[*mstIter]->getFrametype() == LOCKFRAME || *mstIter == frameId) //don't push to existing keyframes and lockframes
          continue; //also ignore mst frame if it's this frame
      //map<int, vector<LimbLabel> > labels;
        std::map<uint32_t, std::vector<LimbLabel> > labels;
        std::map<uint32_t, std::vector<LimbLabel> >::iterator labelPartsIter;

        //check whether parent is a lockframe
        bool parentIsLockframe = false;
        int parentIsSolved = -1;
        if (mstIter != mst.begin())
        {
          if (frames[*mst.parent(mstIter)]->getFrametype() == LOCKFRAME || frames[*mst.parent(mstIter)]->getFrametype() == KEYFRAME)
            parentIsLockframe = true;
          for (uint32_t i = 0; i < allSolves.size(); ++i) //check among accepted solutions
            if (allSolves[i].solvlet.getFrameID() == *mst.parent(mstIter)) //if one fits, use it as prior
              parentIsSolved = i;
        }

        parentIsSolved = -1;

        Frame * lockframe = new Lockframe();

        if (parentIsSolved != -1) //if solved, set the prior to the solve
          lockframe->setSkeleton(allSolves.at(parentIsSolved).solvlet.toSkeleton(frames[frameId]->getSkeleton()));
        else if (parentIsLockframe) //if the parent of this node is a lockframe, use it as a prior
          lockframe->setSkeleton(frames[*mst.parent(mstIter)]->getSkeleton());
        else //otherwise use the root frame as a prior
          lockframe->setSkeleton(frames[frameId]->getSkeleton());


        lockframe->setID(frames[*mstIter]->getID());
        lockframe->setImage(frames[*mstIter]->getImage());
        lockframe->setMask(frames[*mstIter]->getMask());

        //compute the shift between the frame we are propagating from and the current frame
        cv::Point2f shift;
        if (parentIsSolved != -1)
          shift = ism.getShift(allSolves.at(parentIsSolved).solvlet.getFrameID(), frames[*mstIter]->getID());
        else if (parentIsLockframe)
          shift = ism.getShift(frames[*mst.parent(mstIter)]->getID(), frames[*mstIter]->getID());
        else
          shift = ism.getShift(frames[frameId]->getID(), frames[*mstIter]->getID());

        Skeleton skeleton = lockframe->getSkeleton();

        //now set up skeleton params, such as search radius and angle search radius for every part
        //this should very depending on relative distance between frames
        //for each body part
        tree<BodyPart> partTree = skeleton.getPartTree();
        tree<BodyPart>::iterator partIter, parentPartIter;

        //TODO: add check for keyframe or lockframe proximity to angular search radius estimator
        //            bool isBound=false;
        //            //if the frame we are projecting from is close to the frame we are projecting to => restrict angle search distance
        //            //and is the frame we are propagating from, a keyframe? (check only if we disallow bind to lockframes
        //            if(std::abs(frames[*mstIter]->getID()-frames[frameId]->getID())<=anchorBindDistance &&  //check to see whether we are close to a bind frame
        //                    (bindToLockframes || (frames[frameId]->getFrametype()==KEYFRAME))) //and whether we are actually allowed to bind
        //                isBound=true;

        //if so, set bool to true, and get the frame ID that we are close to

        for (partIter = partTree.begin(); partIter != partTree.end(); ++partIter)
        {
          int depth = partTree.depth(partIter);

          float rotationRange = baseRotationRange;//*pow(depthRotationCoeff, depth);
          float searchRange = baseSearchRadius*pow(depthRotationCoeff, depth);

          if (partTree.number_of_children(partIter) == 0)
          {
            searchRange = searchRange * 2;
            rotationRange = rotationRange*depthRotationCoeff;
          }

          partIter->setRotationSearchRange(rotationRange);
          partIter->setSearchRadius(searchRange);
        }
        skeleton.setPartTree(partTree);
        lockframe->setSkeleton(skeleton);

        if (parentIsLockframe)
          frames[*mst.parent(mstIter)]->setSkeleton(skeleton);
        else
          frames[frameId]->setSkeleton(skeleton);

        lockframe->shiftSkeleton2D(shift); //shift the skeleton by the correct amount

        for (uint32_t i = 0; i < detectors.size(); ++i) //for every detector
          labels = detectors[i]->detect(lockframe, params, labels); //detect labels based on keyframe training

        float maxPartCandidates = params.at("maxPartCandidates");

        for (uint32_t i = 0; i < labels.size(); ++i) //for each part
        {
          std::vector<Score> scores = labels[i].at(0).getScores();

          uint32_t isWeak = 0;
          for (uint32_t j = 0; j < scores.size(); ++j)
            if (scores[j].getIsWeak())
              isWeak++;
          std::vector<LimbLabel> tmp;
          for (uint32_t j = 0; j < labels[i].size()*maxPartCandidates; ++j) //for each label that is within the threshold
            tmp.push_back(labels[i].at(j)); //push back the label
          labels[i] = tmp; //set this part's candidates to the new trimmed vector
        }

        std::vector<size_t> numbersOfLabels; //numbers of labels per part

        for (uint32_t i = 0; i < labels.size(); ++i)
          numbersOfLabels.push_back(labels[i].size()); //numbers of labels now contains the numbers

        Space space(numbersOfLabels.begin(), numbersOfLabels.end());
        Model gm(space);

        uint32_t jointFactors = 0, suppFactors = 0, priorFactors = 0;
        //label score cost
        for (partIter = partTree.begin(); partIter != partTree.end(); ++partIter) //for each of the detected parts
        {
          std::vector<Score> scores = labels[partIter->getPartID()].at(0).getScores();
          uint32_t isWeakCount = 0;
          for (uint32_t j = 0; j < scores.size(); ++j)
            if (scores[j].getIsWeak())
              isWeakCount++;

          std::vector<int> varIndices; //create vector of indices of variables
          varIndices.push_back(partIter->getPartID()); //push first value in

          size_t scoreCostShape[] = { numbersOfLabels[partIter->getPartID()] }; //number of labels
          opengm::ExplicitFunction<float> scoreCostFunc(scoreCostShape, scoreCostShape + 1); //explicit function declare

          for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i) //for each label in for this part
          {
            scoreCostFunc(i) = computeScoreCost(labels[partIter->getPartID()].at(i), params); //compute the label score cost
          }

          Model::FunctionIdentifier scoreFid = gm.addFunction(scoreCostFunc); //explicit function add to graphical model
          gm.addFactor(scoreFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
          suppFactors++;

                          //Comment out prior cost factors for the moment
                          //                    ExplicitFunction<float> priorCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

                          //                    //precompute the maxium and minimum for normalisation
                          //                    float priorMin=FLT_MAX, priorMax=0;
                          //                    vector<LimbLabel>::iterator lbl;
                          //                    //vector<float> priorCostFuncValues;
                          //                    for(lbl=labels[partIter->getPartID()].begin(); lbl!=labels[partIter->getPartID()].end(); ++lbl) //for each label in for this part
                          //                    {
                          //                        float val = computePriorCost(*lbl, *partIter, skeleton, params);

                          //                        if(val<priorMin)
                          //                            priorMin = val;
                          //                        if(val>priorMax && val!=FLT_MAX)
                          //                            priorMax = val;

                          //                        //priorCostFuncValues.push_back(val);
                          //                    }

                          //                    //now set up the solutions
                          //                    for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                          //                    {
                          //                        priorCostFunc(i) = computeNormPriorCost(labels[partIter->getPartID()].at(i), *partIter, skeleton, params, priorMin, priorMax);
                          //                    }

                          //                    Model::FunctionIdentifier priorFid = gm.addFunction(priorCostFunc); //explicit function add to graphical model
                          //                    gm.addFactor(priorFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                          //                    priorFactors++;

          if (partIter != partTree.begin()) //if iterator is not on root node, there is always a parent body part
          {
            varIndices.clear();
            parentPartIter = partTree.parent(partIter); //find the parent of this part
            varIndices.push_back(parentPartIter->getPartID()); //push back parent partID as the second variable index
            varIndices.push_back(partIter->getPartID()); //push first value in (parent, this)

            size_t jointCostShape[] = { numbersOfLabels[parentPartIter->getPartID()], numbersOfLabels[partIter->getPartID()] }; //number of labels
            opengm::ExplicitFunction<float> jointCostFunc(jointCostShape, jointCostShape + 2); //explicit function declare

            //first figure out which of the current body part's joints should be in common with the parent body part

            bool toChild = false;
            int pj = parentPartIter->getChildJoint();
            int cj = partIter->getParentJoint();
            if (pj == cj)
            {
              //then current parent is connected to paren
              toChild = true;
            }

            float jointMin = FLT_MAX, jointMax = 0;
            for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i) //for each label in for this part
            {
              for (uint32_t j = 0; j < labels[parentPartIter->getPartID()].size(); ++j)
              {
                float val = computeJointCost(labels[partIter->getPartID()].at(i), labels[parentPartIter->getPartID()].at(j), params, toChild);

                if (val < jointMin)
                  jointMin = val;
                if (val > jointMax && val != FLT_MAX)
                  jointMax = val;
              }
            }

            for (uint32_t i = 0; i < labels[partIter->getPartID()].size(); ++i) //for each label in for this part
            {
              for (uint32_t j = 0; j < labels[parentPartIter->getPartID()].size(); ++j)
              {
                //for every child/parent pair, compute score
                jointCostFunc(j, i) = computeNormJointCost(labels[partIter->getPartID()].at(i), labels[parentPartIter->getPartID()].at(j), params, jointMax, toChild);
              }
            }

            Model::FunctionIdentifier jointFid = gm.addFunction(jointCostFunc); //explicit function add to graphical model
            gm.addFactor(jointFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
            jointFactors++;
          }
        }

        if (debugLevel >= 1)
        {
          float k;
          k = skeleton.getPartTree().size(); //number of bones
          //float expectedSuppFactors=k; //n*k
          //float expectedPriorFactors=k; //n*k
          float expectedJointFactors = (k - 1); //n*(k-1)

          //assert(expectedSuppFactors==suppFactors);
          assert(expectedJointFactors == jointFactors);
          //assert(priorFactors==expectedPriorFactors);
        }
        //            t2 = high_resolution_clock::now();
        //            duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

        //            cerr << "Factor Graph building time "  << duration << endl;

        // set up the optimizer (loopy belief propagation)

        //t1 = high_resolution_clock::now();
        const size_t maxNumberOfIterations = 100;
        const double convergenceBound = 1e-7;
        const double damping = 0.5;
        BeliefPropagation::Parameter parameter(maxNumberOfIterations, convergenceBound, damping);
        BeliefPropagation bp(gm, parameter);

        // optimize (approximately)
        BeliefPropagation::VerboseVisitorType visitor;
        //bp.infer(visitor);
        bp.infer();

        // obtain the (approximate) argmin
        std::vector<size_t> labeling(labels.size());
        bp.arg(labeling);

        std::vector<LimbLabel> solutionLabels;
        for (uint32_t i = 0; i < labels.size(); ++i)
        {
          solutionLabels.push_back(labels[i][labeling[i]]); //pupulate solution vector
        }
        //t2 = high_resolution_clock::now();

        //            duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
        //            cerr << "Factor Graph solving time "  << duration << endl;

        //            t1 = high_resolution_clock::now();

        //labeling now contains the approximately optimal labels for this problem
        Solvlet solvlet(*mstIter, solutionLabels);
        SolvletScore ss;
        ss.solvlet = solvlet;
        if (parentIsSolved != -1)
          ss.parentFrame = allSolves.at(parentIsSolved).solvlet.getFrameID();
        else if (parentIsLockframe)
          ss.parentFrame = frames[*mst.parent(mstIter)]->getID();
        else
          ss.parentFrame = frames[frameId]->getID();
        std::cerr << "done solving!" << std::endl;
        ss.score = evaluateSolution(frames[solvlet.getFrameID()],
          solvlet.getLabels(), params);
        allSolves.push_back(ss);

        //            t2 = high_resolution_clock::now();

        //            duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
        //            cerr << "Solve evaluation time "  << duration << endl;

        delete lockframe; //delete the unused pointer now
      }

      //do detector cleanup
      for (auto i = 0; i < detectors.size(); ++i)
        delete detectors[i];
      detectors.clear();
      ignore.push_back(frames[frameId]->getID()); //add this frame to the ignore list for future iteration, so that we don't propagate from it twice
    }
    
    return allSolves;
  }

  int NSKPSolver::test(int frameId, const std::vector<Frame *> &frames, std::map<std::string, float> params, const ImageSimilarityMatrix& ism, const std::vector<MinSpanningTree>& trees, std::vector<int> &ignore)
  {
    return 0;
  }

  std::vector<Solvlet> NSKPSolver::propagateKeyframes(std::vector<Frame*>& frames, std::map<std::string, float> params, const ImageSimilarityMatrix& ism, const std::vector<MinSpanningTree>& trees, std::vector<int>& ignore)
  {
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    if (frames.size() == 0)
      return std::vector<Solvlet>();

    std::vector<std::vector<SolvletScore> > allSolves;

    for (uint32_t i = 0; i < frames.size(); ++i)
      allSolves.push_back(std::vector<SolvletScore>()); //empty vector to every frame slot

    std::vector<Lockframe*> lockframes;

    //build frame MSTs by ID's as in ISM
    //vector<MinSpanningTree> trees = buildFrameMSTs(ism, params);
    //now add variables to the space, with number of detections

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

    //    vector<future<vector<NSKPSolver::SolvletScore> > > futures;

    std::cerr << "Set-up time " << duration << std::endl;

    // //THIS IS THE ASYNC VERSION, THAT NEEDS DEBUGGING  -------------------------------------------------------------
//    vector<future<vector<SolvletScore> > > futures;
//    for (uint32_t frameId = 0; frameId < frames.size(); ++frameId)
//    {
//        int captureIndex = frameId;
//        //[&] { a.foo(100); }
//        //futures.push_back(std::async([&] {this->test(frameId, frames, params, ism, trees, ignore);}));
//        futures.push_back(async(std::launch::async | std::launch::deferred,[=, &ignore]()->vector < NSKPSolver::SolvletScore >
//        {return propagateFrame(captureIndex, frames, params, ism, trees, ignore); }));
//    }

//    cerr << "Launched " << futures.size() << " threads." << endl;

//    vector<vector<SolvletScore> > temp;
//    for (auto &e : futures) {
//        ;
//        try {
//            //e.wait();
//            temp.push_back(e.get());
//            //std::cout << "You entered: " << x << '\n';
//        }
//        catch (std::exception&) {
//            std::cout << "[exception caught]";
//        }
//        //         if(solves.size()>0)
//        //             allSolves[solves[0].solvlet.getFrameID()]=solves;
//    }
    // //END -----------------------------------------------------------------------------

    //THE SINGLE-THREAD VERSION ----------------------------------------------------------
    std::vector<std::vector<SolvletScore> > temp;
    for (uint32_t frameId = 0; frameId < frames.size(); ++frameId)
    {
      temp.push_back(propagateFrame(frameId, frames, params, ism, trees, ignore));
    }

    for (uint32_t i = 0; i < temp.size(); ++i)
    {
      if (temp[i].size()>0)
        allSolves[temp[i].at(0).solvlet.getFrameID()] = temp[i];
    }
    //END --------------------------------------------------------------------------------

    //now extract the best solves
    std::map<uint32_t, SolvletScore> bestSolves;

    float acceptLockframeThreshold = params.at("nskpLockframeThreshold");

    for (auto frameSolves : allSolves)
    {
      for (auto solve : frameSolves)
      {
        if (solve.score >= acceptLockframeThreshold)
        {
          bestSolves.emplace(solve.solvlet.getFrameID(), solve); //emplace this solve if one isn't in there yet

          if (solve.score > bestSolves.at(solve.solvlet.getFrameID()).score) //now compare to solve in the map
            bestSolves.at(solve.solvlet.getFrameID()) = solve; //replace with this one if it's better
        }
      }
    }

    //now create skeletons for these solves

    for (auto bs : bestSolves)
    {
      SolvletScore ss = bs.second;
      int thisFrameID = ss.solvlet.getFrameID();
      int parentFrameID = ss.parentFrame;//the ID of the frame the solve prior came from
      
      Lockframe * lockframe = new Lockframe();
      lockframe->setImage(frames[thisFrameID]->getImage());
      lockframe->setMask(frames[thisFrameID]->getMask());
      lockframe->setID(thisFrameID);
      Skeleton parentSkel = frames[parentFrameID]->getSkeleton();
      Skeleton skel = ss.solvlet.toSkeleton(parentSkel);
      //parent skeleton is the basis, this will copy scale and other params, but have different locations
      lockframe->setSkeleton(skel);
      lockframe->setParentFrameID(parentFrameID);

      lockframes.push_back(lockframe);
    }

    std::vector<Solvlet> solvlets;

    for (uint32_t i = 0; i < lockframes.size(); ++i)
    {
      if (frames[lockframes[i]->getID()]->getFrametype() != LOCKFRAME
        && frames[lockframes[i]->getID()]->getFrametype() != KEYFRAME) //never replace keyframes and existing lockframes
      {
        delete frames[lockframes[i]->getID()]; //delete the frame currently there, and replace with lockframe
        frames[lockframes[i]->getID()] = lockframes.at(i); //make pointers point to the correct objects
        solvlets.push_back(bestSolves.at(lockframes.at(i)->getID()).solvlet);
      } //unless this lockframe replaced something in the original vector, delte it
      else
      {
        delete lockframes[i];
      }
    }

    std::cerr << "Generated " << lockframes.size() << " lockframes!" << std::endl;

    return solvlets;
  }

  //return the index of the first instance of frame with matching id
  //if no matching id in vector, return -1
  uint32_t NSKPSolver::findFrameIndexById(int id, std::vector<Frame*> frames)
  {
    for (uint32_t i = 0; i < frames.size(); ++i)
    {
      if (frames[i]->getID() == id)
        return i;
    }
    return -1;
  }

  //compute label score
  float NSKPSolver::computeScoreCost(const LimbLabel& label, std::map<std::string, float> params)
  {
    //    if(label.getIsOccluded() )// || label.getIsWeak()) //if it's occluded, return zero
    //        return 0;

    std::string hogName = "18500";
    std::string csName = "4409412";
    std::string surfName = "21316";

    params.emplace("imageCoeff", 1.0);
    params.emplace("useCSdet", 1.0);
    params.emplace("useHoGdet", 0.0);
    params.emplace("useSURFdet", 0.0);

    float lambda = params.at("imageCoeff");

    //@FIX
    float useHoG = params.at("useHoGdet");
    float useCS = params.at("useCSdet");
    float useSURF = params.at("useSURFdet");

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
      if (scores[i].getScore() == -1.0)//if score is -1, set it to 1
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
  float NSKPSolver::computeJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, bool toChild)
  {
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

  //compute distance to parent limb label
  float NSKPSolver::computeNormJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, float max, bool toChild)
  {
    //emplace default
    params.emplace("jointCoeff", 0.5);
    params.emplace("jointLeeway", 0.05);
    params.emplace("debugLevel", 1);

    //read params
    float lambda = params.at("jointCoeff");
    int debugLevel = params.at("debugLevel");

    //float leeway = params.at("jointLeeway");
    cv::Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    child.getEndpoints(c0, c1);
    parent.getEndpoints(p0, p1);

    //child length
    //float clen = sqrt(pow(c0.x-c1.x, 2)+pow(c0.y-c1.y, 2));

    //normalise this?
    //give some leeway
    float score = 0;
    if (toChild) //connected to parent's child joint
      score = sqrt(pow(c0.x - p1.x, 2) + pow(c0.y - p1.y, 2)) / max;
    else //else connected to parent's parent joint
      score = sqrt(pow(c0.x - p0.x, 2) + pow(c0.y - p0.y, 2)) / max;
    //    if(score<(clen*leeway)/max) //any distnace below leeway is zero
    //        score=0;
    //return the squared distance from the lower parent joint p1, to the upper child joint c0

    //output a sentence about who connected to whom and what the score was
    if (debugLevel >= 1 && (child.getLimbID() == 7 || child.getLimbID() == 6) && !toChild)
      std::cerr << "Part " << child.getLimbID() << " is connecting to part " << parent.getLimbID() << " PARENT joint" << std::endl;

    return lambda*score;
  }

  //compute distance to the body part prior
  float NSKPSolver::computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params)
  {
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

  float NSKPSolver::computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params, float min, float max)
  {
    params.emplace("priorCoeff", 0.0);
    float lambda = params.at("priorCoeff");
    cv::Point2f p0, p1, pp0, pp1;
    label.getEndpoints(p0, p1);
    pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
    pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

    //normalise this cost?

    //return the sum of squared distances between the corresponding joints, prior to label
    return lambda*((pow((p0.x - pp0.x), 2) + pow((p0.y - pp0.y), 2) + pow((p1.x - pp1.x), 2) + pow((p1.y - pp1.y), 2)) / max);
  }

  //build an MST for every frame and return the vector
  std::vector<MinSpanningTree > NSKPSolver::buildFrameMSTs(const ImageSimilarityMatrix &ism, std::map<std::string, float> params) //int treeSize, float threshold)
  {
    //emplace defaults
    params.emplace("treeSize", ism.size()); //no size limit

    //float mean = ism.mean();
    float sd = ism.stddev();
    float min = ism.min();

    //cout << "ISM Mean " << mean << " sd " << sd << " min " << min << endl;

    //cout << "The min is " << (mean-min)/sd << " deviations away from mean. " << endl;
    //cout << "One sd is " << sd/min << " of min." << endl;

    float simThreshD = 1.0 + 3.5*sd / min;

    //cout << "Setting simThresh to " << simThresh << endl;

    params.emplace("mstThresh", simThreshD); //set similarity as multiple of minimum, MUST be >=1

    //params.emplace("mstThresh", 2.5); //set similarity as multiple of minimum, MUST be >=1
    int treeSize = params.at("treeSize");
    float simThresh = params.at("mstThresh");
    std::vector<MinSpanningTree> frameMST;
    for (uint32_t i = 0; i < ism.size(); ++i)
      frameMST.push_back(MinSpanningTree());

    //vector<vector<int> > frameMSTvec;

    for (uint32_t i = 0; i < ism.size(); ++i)
    {
      //for each frame, build an MST
      MinSpanningTree frameTree(ism, i, treeSize, simThresh);
      //and add to vector
      frameMST[i] = frameTree;
      std::cout << i << " MST built" << std::endl;
    }

    return frameMST;
  }

  //suggest maximum number of keyframes
  //function should return vector with suggested keyframe numbers
  std::vector<cv::Point2i> NSKPSolver::suggestKeyframes(const ImageSimilarityMatrix& ism, std::map<std::string, float> params)
  {
    auto debugLevel = params.at("debugLevel");
    //if(params.at("debugLevel")>=1)
    std::cerr << "Building all MSTs..." << std::endl;
    std::vector<MinSpanningTree> mstVec = buildFrameMSTs(ism, params);
    //if(debugLevel>=1)
    std::cerr << "Finished building MSTs" << std::endl;
    params.emplace("minKeyframeDist", 1); //don't suggest keyframes that are too close together
    int minKeyframeDist = params.at("minKeyframeDist");
    std::vector<std::vector<uint32_t> > orderedList;
    for (uint32_t i = 0; i < mstVec.size(); ++i)
    {
      tree<int>::iterator iter;
      tree<int> MST = mstVec[i].getMST();

      std::vector<uint32_t> frames;

      for (iter = MST.begin(); iter != MST.end(); ++iter)
        frames.push_back(*iter); //create a vector of vectors
      orderedList.push_back(frames);
    }
    //this is the simple way of counting the number of frames that made it in
    //alternatively we could come up with a more complex scheme for determining keyframe optimality
    std::vector<cv::Point2i> frameOrder;
    while (orderedList.size() != 0)
    {
      //find the largest frame MST:
      uint32_t maxSize = 0;
      int idx = -1;
      for (uint32_t i = 0; i < orderedList.size(); ++i)
      {
        if (orderedList[i].size() > maxSize)
        {
          idx = i;
          maxSize = orderedList[i].size();
        }
      }
      //if there largest vector remaining is of legnth zero, stop
      if (maxSize <= 1)
        break;


      //store it as the best candidate
      std::vector<uint32_t> erasedVector = orderedList[idx];

      frameOrder.push_back(cv::Point2i(erasedVector[0], maxSize));

      orderedList.erase(orderedList.begin() + idx);

      //remove all values in that vector from all others
      for (uint32_t i = 0; i < orderedList.size(); ++i)
        //for each element in erasedVector
        for (uint32_t j = 0; j < erasedVector.size(); ++j)
          orderedList[i].erase(std::remove(orderedList[i].begin(), orderedList[i].end(), erasedVector[j]), orderedList[i].end());
    }
    //now tidy up the frame order by forcing minimum keyframe distance
    std::vector<cv::Point2i> aux;
    aux.push_back(frameOrder[0]);
    for (std::vector<cv::Point2i>::iterator i = frameOrder.begin(); i != frameOrder.end(); ++i)
    {
      int isOk = true;
      int thisFrame = i->x;
      for (std::vector<cv::Point2i>::iterator j = aux.begin(); j != aux.end(); ++j)
      {
        int thatFrame = j->x;

        if (std::abs(thisFrame - thatFrame) < minKeyframeDist)
        {
          isOk = false;
          break;
        }
      }

      if (isOk) //if it's ok so far, add it to final
        aux.push_back(*i);
    }
    //return the resulting vector
    
    return aux;
  }

  float NSKPSolver::evaluateSolution(Frame* frame, std::vector<LimbLabel> labels, std::map<std::string, float> params)
  {
    // /*
    //   There should clearly be several factors that affect the outcome of an evaluation:
    //   1) Mask coverage
    //   2) Parts falling outside mask range
    //   3) ?

    //   */

    //engaged pixles - the total number of pixels that are either within a limb label of within the mask
    //correct pixels - those pixles that are black and outside a label, and white and inside a label
    //incorrect pixels - those pixels that are black and inside a label, and white and outside a label
    //score = correct/(correct+incorrect)

    //emplace defaults
    params.emplace("badLabelThresh", 0.52); //if less than 52% of the pixels are in the mask, label this label bad
    params.emplace("debugLevel", 1);
    params.emplace("maxFrameHeight", 288);  //emplace if not defined

    int maxFrameHeight = params.at("maxFrameHeight");
    int debugLevel = params.at("debugLevel");

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
    float badLabelThresh = params.at("badLabelThresh");

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

    if (debugLevel >= 1)
      for (std::vector<cv::Point2f>::iterator badL = badLabelScores.begin(); badL != badLabelScores.end(); ++badL)
        std::cerr << "Part " << badL->x << " is badly localised, with score " << badL->y << std::endl;

    if (badLabelScores.size() != 0) //make the solution eval fail if a part is badly localised
      solutionEval = solutionEval - 1.0;

    if (debugLevel >= 1)
      std::cerr << "Solution evaluation score - " << solutionEval << " for frame " << frame->getID() << " solve from " << frame->getParentFrameID() << std::endl;

    return solutionEval;
  }

}
