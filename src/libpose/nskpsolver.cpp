#include <limits>
#include "nskpsolver.hpp"
#include <tree.hh>
#include "lockframe.hpp"
#include "colorHistDetector.hpp"
#include "hogDetector.hpp"
#include "tlpssolver.hpp"

//using namespace opengm;

NSKPSolver::NSKPSolver()
{
	id = 1; //this should be unique
	name = "NSKP"; //this should be unique
}

NSKPSolver::~NSKPSolver() //inherited virtual
{

}

vector<Solvlet> NSKPSolver::solve(Sequence& sequence) //inherited virtual
{
	map<string, float> params; //set the default parameters vector

    //set some parameter defaults
	
	//pass to next level solve function, for ISM computing
    return this->solve(sequence, params);
}

vector<Solvlet> NSKPSolver::solve(Sequence& sequence, map<string, float> params) //inherited virtual
{
	//compute the ISM here
    ImageSimilarityMatrix ISM(sequence.getFrames());

	//pass to solve function
    return this->solve(sequence, params, ISM);
}

vector<Solvlet> NSKPSolver::solve(Sequence& sequence, map<string, float>  params, const ImageSimilarityMatrix& ism) //inherited virtual
{
	//parametrise the number of times frames get propagated
    params.emplace("nskpIters", 0); //set number of iterations, 0 to iterate until no new lockframes are introduced

    uint32_t nskpIters=params.at("nskpIters");
    if(nskpIters==0)
        nskpIters=INT32_MAX;

    sequence.computeInterpolation(params); //interpolate the sequence first
	//propagate keyframes
    vector<Frame*> propagatedFrames = sequence.getFrames();
    uint32_t lockframesLastIter=0;
    for(uint32_t i=0; i<nskpIters; ++i)
    {
        if(i>=nskpIters)
            break;

        propagateKeyframes(propagatedFrames, params, ism);

        //calculate number of lockframes in the sequence
        uint32_t numLockframes=0;
        for(vector<Frame*>::iterator frameIter=propagatedFrames.begin(); frameIter!=propagatedFrames.end(); ++frameIter)
        {
            if((*frameIter)->getFrametype()==LOCKFRAME)
                numLockframes++;
        }

        if(numLockframes==lockframesLastIter) //terminate loop if no more lockframes are generated
        {
            cerr << "Terminating keyframe propagation after " << i << " iterations." << endl;
            break;
        }
        lockframesLastIter=numLockframes;
    }

	//create tlps solver
    TLPSSolver tlps;
    //sequence.setFrames(propagatedFrames);

	//the params map should countain all necessary parameters for solving, if they don't exist, default values should be used

	//return the TLPS solve
    return tlps.solve(sequence, params);
}

void NSKPSolver::propagateKeyframes(vector<Frame*>& frames, map<string, float>  params, const ImageSimilarityMatrix& ism)
{
	// //@Q should frame ordering matter? in this function it should not matter, so no checks are necessary
	// float mst_thresm_multiplier=params.at("mst_thresh_multiplier"); //@FIXME PARAM this is a param, not static
	// int mst_max_size=params.at("mst_max_size"); //@FIXME PARAM this is a param, not static

    //the params vector should contain all necessary parameters, if a parameter is not present, default values should be used

    int numLockframesGenerated=0;
    params.emplace("useHoGdet", 1); //determine if HoG descriptor is used and with what coefficient
    params.emplace("useCSdet", 0); //determine if colorhist detector is used and with what coefficient
    params.emplace("useSURFdet", 0); //determine whether SURF detector is used and with what coefficient
    params.emplace("maxPartCandidates", 200); //set the max number of part candidates to allow into the solver
    params.emplace("acceptLockframeThreshold", 0.5); //set up the lockframe accept threshold by mask coverage
    params.emplace("debugLevel", 1); //set up the lockframe accept threshold by mask coverage
    params.emplace("propagateToLockframes", 0);

    uint32_t maxPartCandidates = params.at("maxPartCandidates");
    float useHoG = params.at("useHoGdet");
    float useCS = params.at("useHoGdet");
    uint32_t debugLevel = params.at("debugLevel");
    bool propagateToLockframes=params.at("propagateToLockframes");

    vector<Frame*> lockframes;

	//build frame MSTs by ID's as in ISM
	vector<MinSpanningTree> trees = buildFrameMSTs(ism, params);
	//now add variables to the space, with number of detections
	for(uint32_t frameId=0; frameId<frames.size(); ++frameId)
	{
		if(frames[frameId]->getFrametype()!=0x02) //as long as it's not an interpolated frame, try to propagate from it
		{
			tree<int> mst = trees[frames[frameId]->getID()].getMST(); //get the MST, by ID, as in ISM
			tree<int>::iterator mstIter;
			//do OpenGM solve for single factor graph

            vector<Detector*> detectors;
            if(useHoG)
                detectors.push_back(new HogDetector());
            if(useCS)
                detectors.push_back(new ColorHistDetector());

			vector<Frame*> trainingFrames;
			trainingFrames.push_back(frames[frameId]); //set training frame by index
			
            for(uint32_t i=0; i<detectors.size(); ++i)
            {
                detectors[i]->train(trainingFrames, params);
            }

			for(mstIter=mst.begin(); mstIter!=mst.end(); ++mstIter) //for each frame in the MST
			{
                if(frames[*mstIter]->getFrametype()==KEYFRAME) //skip keyframes
                    continue;
                else if(frames[*mstIter]->getFrametype()==LOCKFRAME && !propagateToLockframes) //skip lockframes, unless
                    continue;
                vector<vector<LimbLabel> > labels, tempLabels;
                vector<vector<LimbLabel> >::iterator labelPartsIter;

                for(uint32_t i=0; i<detectors.size(); ++i) //for every detector
                {
                    labels = detectors[i]->detect(frames[*mstIter], params, labels); //detect labels based on keyframe training
                }

                for(labelPartsIter=labels.begin();labelPartsIter!=labels.end();++labelPartsIter) //now take the top labels
                {
                    vector<LimbLabel> temp;
                    for(uint32_t currentSize=0; currentSize<maxPartCandidates && currentSize<labelPartsIter->size(); ++currentSize)
                    {
                        temp.push_back(labelPartsIter->at(currentSize));
                    }
                    tempLabels.push_back(temp);
                }

                labels = tempLabels;

				vector<size_t> numbersOfLabels; //numbers of labels per part

				for(uint32_t i=0; i<labels.size(); ++i)
				{
					numbersOfLabels.push_back(labels[i].size());
				} //numbers of labels now contains the numbers

				Space space(numbersOfLabels.begin(), numbersOfLabels.end());
				Model gm(space);

				Skeleton skeleton = frames[*mstIter]->getSkeleton();
				tree<BodyPart> partTree = skeleton.getPartTree();
				tree<BodyPart>::iterator partIter, parentPartIter;

                uint32_t jointFactors=0, suppFactors=0, priorFactors=0;
				//label score cost
				for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter) //for each of the detected parts
				{
					vector<int> varIndices; //create vector of indices of variables
					varIndices.push_back(partIter->getPartID()); //push first value in

					size_t scoreCostShape[]={numbersOfLabels[partIter->getPartID()]}; //number of labels
					ExplicitFunction<float> scoreCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

					for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
					{
						scoreCostFunc(i) = computeScoreCost(labels[partIter->getPartID()].at(i), params); //compute the label score cost
					}

					Model::FunctionIdentifier scoreFid = gm.addFunction(scoreCostFunc); //explicit function add to graphical model
					gm.addFactor(scoreFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    suppFactors++;

					ExplicitFunction<float> priorCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

					for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
					{
						priorCostFunc(i) = computePriorCost(labels[partIter->getPartID()].at(i), *partIter, skeleton, params);
					}

					Model::FunctionIdentifier priorFid = gm.addFunction(priorCostFunc); //explicit function add to graphical model
					gm.addFactor(priorFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    priorFactors++;
					
					if(partIter!=partTree.begin()) //if iterator is not on root node, there is always a parent body part
					{
                        varIndices.clear();
                        parentPartIter=partTree.parent(partIter); //find the parent of this part
                        varIndices.push_back(parentPartIter->getPartID()); //push back parent partID as the second variable index
                        varIndices.push_back(partIter->getPartID()); //push first value in (parent, this)

                        size_t jointCostShape[]={numbersOfLabels[parentPartIter->getPartID()], numbersOfLabels[partIter->getPartID()]}; //number of labels
						ExplicitFunction<float> jointCostFunc(jointCostShape, jointCostShape+2); //explicit function declare

						for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
						{
							for(uint32_t j=0; j<labels[parentPartIter->getPartID()].size(); ++j)
							{
								//for every child/parent pair, compute score
                                jointCostFunc(j, i) = computeJointCost(labels[partIter->getPartID()].at(i), labels[parentPartIter->getPartID()].at(j), params);
							}
                        }

						Model::FunctionIdentifier jointFid = gm.addFunction(jointCostFunc); //explicit function add to graphical model
						gm.addFactor(jointFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                        jointFactors++;
					}
				}

                if(debugLevel>=1)
                {
                    float k;
                    k=skeleton.getPartTree().size(); //number of bones
                    float expectedSuppFactors=k; //n*k
                    float expectedPriorFactors=k; //n*k
                    float expectedJointFactors=(k-1); //n*(k-1)

                    assert(expectedSuppFactors==suppFactors);
                    assert(expectedJointFactors==jointFactors);
                    assert(priorFactors==expectedPriorFactors);
                }

				// set up the optimizer (loopy belief propagation)

			   	const size_t maxNumberOfIterations = 40;
			   	const double convergenceBound = 1e-7;
			   	const double damping = 0.5;
			   	BeliefPropagation::Parameter parameter(maxNumberOfIterations, convergenceBound, damping);
			   	BeliefPropagation bp(gm, parameter);
				
			   // optimize (approximately)
				BeliefPropagation::VerboseVisitorType visitor;
				bp.infer(visitor);

				// obtain the (approximate) argmin
				vector<size_t> labeling(labels.size());
				bp.arg(labeling);

				vector<LimbLabel> solutionLabels;
				for(uint32_t i=0; i<labels.size();++i)
				{
					solutionLabels.push_back(labels[i][labeling[i]]); //pupulate solution vector
				}
				//labeling now contains the approximately optimal labels for this problem
                float solutionScore = evaluateSolution(frames[*mstIter], solutionLabels, params);
				//evaluate the solution, and decide whether it should be added to keyframes

                float acceptLockframeThreshold = params.at("acceptLockframeThreshold"); //@PARAM this is a parameter, not a static value
				if(solutionScore<=acceptLockframeThreshold)
				{
					//set up the lockframe
					Solvlet solvlet(*mstIter, solutionLabels);
					Skeleton skel(solvlet.toSkeleton());
                    skel.setScale(frames[*mstIter]->getSkeleton().getScale()); //set skeleton scale
                    Frame * lockframe = new Lockframe();

                    lockframe->setSkeleton(skel);
                    lockframe->setID(frames[*mstIter]->getID());

                    lockframe->setImage(frames[*mstIter]->getImage());
                    lockframe->setMask(frames[*mstIter]->getMask());
                    lockframe->setSkeleton(skel);

                    //frames[*mstIter]=&lockframe;
                    lockframes.push_back(lockframe);
                    numLockframesGenerated++;
				}
			}
		}
	}

    for(uint32_t i=0; i<lockframes.size();++i)
    {
        *(frames[lockframes[i]->getID()]) = *(lockframes.at(i)); //make pointers point to the correct objects
    }

    cerr << "Generated " << numLockframesGenerated << " lockframes!" << endl;

//	vector<Frame*> returnFrames;
//	//look at lockframes and frames, and put together the return frames
//	for(uint32_t i=0; i<frames.size(); ++i)
//	{
//		int lockframeIndex = findFrameIndexById(frames[i]->getID(), frames);
//		if(lockframeIndex>=0) //if a new lockframe exists at this id
//		{
//			//push it into the sequen
//			returnFrames.push_back(lockframes[lockframeIndex]);
//		}
//		else //otherwise push back the old frame
//		{
//			returnFrames.push_back(frames[i]);
//		}
//	}
//	return returnFrames;
}

//return the index of the first instance of frame with matching id 
//if no matching id in vector, return -1
uint32_t NSKPSolver::findFrameIndexById(int id, vector<Frame*> frames)
{
	for(uint32_t i=0; i<frames.size(); ++i)
	{
		if(frames[i]->getID()==id)
			return i;
	}
	return -1;
}

//compute label score
float NSKPSolver::computeScoreCost(const LimbLabel& label, map<string, float> params)
{
    params.emplace("imageCoeff", 0.5);
    params.emplace("scoreIndex", 0);
	//emplace first
	float lambda = params.at("imageCoeff");
    float scoreIndex = params.at("scoreIndex");
	//@FIX
	//for now, just return the first available score
	vector<Score> scores = label.getScores();
	if(scores.size()>0)
		return lambda*scores[scoreIndex].getScore();
	else //if no scores present return -1
		return -1;
}

//compute distance to parent limb label
float NSKPSolver::computeJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params)
{
	//emplace default
    params.emplace("jointCoeff", 0.5);
	float lambda = params.at("jointCoeff");
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	child.getEndpoints(c0,c1);
	parent.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return lambda*(pow((c0.x-p1.x), 2)+pow((c0.y-p1.y), 2));
}

//compute distance to the body part prior
float NSKPSolver::computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, map<string, float> params)
{
    params.emplace("priorCoeff", 0.5);
	float lambda = params.at("priorCoeff");
	Point2f p0,p1, pp0, pp1;
	label.getEndpoints(p0,p1);
	pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();	
	pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

	//return the sum of squared distances between the corresponding joints, prior to label
	return lambda*(pow((p0.x-pp0.x), 2)+pow((p0.y-pp0.y), 2)+pow((p1.x-pp1.x), 2)+pow((p1.y-pp1.y), 2));
}



//build an MST for every frame and return the vector
vector<MinSpanningTree > NSKPSolver::buildFrameMSTs(ImageSimilarityMatrix ism, map<string, float> params) //int treeSize, float threshold)
{
	//emplace defaults
    params.emplace("treeSize", ism.size()); //no size limit
    params.emplace("simThresh", 2.5); //set similarity as multiple of minimum, MUST be >=1
	int treeSize = params.at("treeSize");
	float simThresh = params.at("simThresh");
    vector<MinSpanningTree> frameMST;
    for(uint32_t i=0; i<ism.size(); ++i)
    {
        frameMST.push_back(MinSpanningTree());
    }

    //vector<vector<int> > frameMSTvec;

    for(uint32_t i=0; i<ism.size(); ++i)
    {
    	//for each frame, build an MST
    	MinSpanningTree frameTree(ism, i, treeSize, simThresh);
    	//and add to vector
        frameMST[i] = frameTree;
    }

    return frameMST;
}

//suggest maximum number of keyframes
//function should return vector with suggested keyframe numbers
vector<Point2i> NSKPSolver::suggestKeyframes(vector<MinSpanningTree>& mstVec, map<string, float> params)
{
	vector<vector<uint32_t> > orderedList;
	for(uint32_t i=0; i<mstVec.size(); ++i)
	{
	    tree<int>::iterator iter;
        tree<int> MST = mstVec[i].getMST();	
        
        vector<uint32_t> frames;

        for(iter=MST.begin(); iter!=MST.end(); ++iter)
        {
            frames.push_back(*iter); //create a vector of vectors
        }
        orderedList.push_back(frames);
    }
    //this is the simple way of counting the number of frames that made it in
    //alternatively we could come up with a more complex scheme for determining keyframe optimality
    vector<Point2i> frameOrder;
    while(orderedList.size()!=0)
    {
        //find the largest frame MST:
        uint32_t maxSize=0;
        uint32_t idx=-1;
        for(uint32_t i=0; i<orderedList.size(); ++i)
        {
            if(orderedList[i].size()> maxSize)
            {
                idx = i;
                maxSize = orderedList[i].size();
            }
        }
        //if there largest vector remaining is of legnth zero, stop
        if(maxSize==1)
            break;


        //store it as the best candidate
        vector<uint32_t> erasedVector = orderedList[idx];

        frameOrder.push_back(Point2i(erasedVector[0], maxSize));

        orderedList.erase(orderedList.begin() + idx);

        //remove all values in that vector from all others
        for(uint32_t i=0; i<orderedList.size(); ++i)
        {
            //for each element in erasedVector
            for(uint32_t j=0; j<erasedVector.size(); ++j)
            {
                orderedList[i].erase(std::remove(orderedList[i].begin(), orderedList[i].end(), erasedVector[j]), orderedList[i].end());
            }
        }
    }
    //return the resulting vector
    return frameOrder;
}

float NSKPSolver::evaluateSolution(Frame* frame, vector<LimbLabel> labels, map<string, float> params)
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

    Mat mask = frame->getMask();
    int correctPixels=0, incorrectPixels=0;

    for(uint32_t i=0; i<mask.rows; ++i) //at every row
    {
        for(uint32_t j=0; j<mask.cols; ++j) //and every col
        {
            //check whether pixel hit a label
            bool labelHit=false;
            for(vector<LimbLabel>::iterator label=labels.begin(); label!=labels.end(); ++label)
            {
                if(label->containsPoint(Point2f(i,j)))
                {
                    labelHit=true;
                    break;
                }
            }

            //check pixel colour
            int intensity = mask.at<uchar>(i, j);
            bool blackPixel=intensity<10;


            if(blackPixel && labelHit) //if black in label, incorrect
                incorrectPixels++;
            else if(!blackPixel && !labelHit) //if white not in label, incorret
                incorrectPixels++;
            else //otherwise correct
                correctPixels++;
        }
    }

    return correctPixels/(correctPixels+incorrectPixels);

    // float numPixels=0; //total number of pixels in mask
    // float numHitPixels=0; //number of pixels in polygon that are inside the mask
    // float numMissPixels=0; //number of pixels in polygons that are outside of the mask
    // Mat maskMat = frame->getMask();
    // // //we are at some frame with the image already loaded (currentFrameNumber)

    // //count the total number of mask pixels
    // for(int x=0; x<maskMat.rows(); ++x)
    // {
    //     for(int y=0; y<maskMat.cols(); ++y)
    //     {
    //     	//@FIXME need check to see whether maskMat is really of type uchar
    //         int intensity = maskMat.at<uchar>(y, x);
    //         bool blackPixel=intensity<10; //mask pixel intensity threshold below which we consider it to be black

    //         //QColor maskCol = mask.pixel(x,y);
    //         if(!blackPixel) //only take in pixels that are in the mask
    //         {
    //             numPixels++; //count the total number of pixels
    //         }
    //     }
    // }

    //counted the total number of pixels in the mask
    //this can be compared to the number of

    // vector<float> limbScores;
    // for(int numLabel=0; numLabel<labels.size(); ++numLabel)
    // {
    //     float pixHit=0;
    //     float pixTotal=0;

    //     LimbLabel label = labels[numLabel];

    //     for(int x=0; x<maskMat.rows(); ++x)
    //     {
    //         for(int y=0; y<maskMat.cols(); ++y)
    //         {
    //         	//@NEEDS FIXING
    //             if(label.containsPoint(Point2f(x,y))) //polygon from label) //only take in pixels that are in the mask
    //             {
    //                 pixTotal++;

    //                 int intensity = maskMat.at<uchar>(y, x);

    //                 bool blackPixel=intensity<10; //if all intensities are zero

    //                 //QColor maskCol = mask.pixel(x,y);
    //                 if(!blackPixel)
    //                 {
    //                     pixHit++;
    //                 }
    //             }
    //         }
    //     }

    //     limbScores.push_back(pixHit/pixTotal);
    // }


    // for(int x=0; x<mask.width(); ++x)
    // {
    //     for(int y=0; y<mask.height(); ++y)
    //     {

    //         int intensity = maskMat.at<uchar>(y, x);

    //         bool blackPixel=intensity<10; //if all intensities are zero
    //         //QColor maskCol = mask.pixel(x,y);

    //         if(!blackPixel) //only take in pixels that are in the mask
    //         {
    //             numPixels++; //count the total numbe rf pixels
    //             bool hit=false;
    //             for(int i=0; i<labels.size(); ++i)
    //             {

    //                 LimbLabel label = labels[i];
    //                 if(label.containsPoint(Point2f(x,y))) //polygon from label
    //                 {
    //                     hit = true;
    //                     break;
    //                 }
    //             }
    //             if(hit)
    //             {
    //                 numHitPixels++;
    //             }
    //         }
    //         else
    //         {
    //             bool hit=false;
    //             for(int i=0; i<labels.size(); ++i)
    //             {
    //                 LimbLabel label = labels[i];

    //                 if(label.containsPoint(Point2f(x,y))) //polygon from label
    //                 {
    //                     hit = true;
    //                     break;
    //                 }
    //             }
    //             if(hit)
    //             {
    //                 //colour this pixel in as hit
    //                 numMissPixels++;
    //             }
    //         }
    //     }
    // }

    // float avgSuppScore=0;

    // for(int i=0; i<labels.size();++i)
    // {
    //     if(i==0 || i>5) //count only the real labels
    //         avgSuppScore+=labels[i].supportScore;
    // }

    // avgSuppScore=avgSuppScore/labels.size();

    // //show the worst half of labels
    // //    cerr << "Poorly localised labels: ";
    // //    for(int i=0; i<labels.size(); ++i)
    // //    {
    // //        if((i==0 || i>5) && labels[i].supportScore>avgSuppScore) //high is bad
    // //            cerr << limbName(i).toStdString() << " (" <<labels[i].supportScore<<") ";
    // //    }
    // //    cerr << endl;

    // // cerr << "Poorly localised labels (by hit/total pixels ratio): ";
    // // for(int i=0; i<labels.size(); ++i)
    // // {
    // //     if((i==0 || i>5) && limbScores[i]<0.4) //high is bad
    // //         cerr << limbName(i).toStdString() << " (" <<limbScores[i]<<") ";
    // // }
    // // cerr << endl;

    // //also, if there are any completely broken limbs, this should lower the score

    // //@FIXME needs testing to establish that this scheme makes sense
    // return (numHitPixels-numMissPixels)/numPixels;
}
