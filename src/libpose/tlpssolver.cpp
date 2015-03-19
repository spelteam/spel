#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "tlpssolver.hpp"
#include "lockframe.hpp"
#include "colorHistDetector.hpp"
#include "hogDetector.hpp"
#include "math.h"
#include "interpolation.hpp"

using namespace Eigen;

TLPSSolver::TLPSSolver(void)
{
	id=0;
	name="TLPS";
}

TLPSSolver::~TLPSSolver(void)
{

}

vector<Solvlet> TLPSSolver::solve(Sequence &frames) //inherited virtual
{
	map<string, float> params; //set the default parameters vector

	//set some parameter deefaults
	params.at("scoreIndex")=0;
	
	//pass to next level solve function, for ISM computing

	return this->solve(frames, params);
}

vector<Solvlet> TLPSSolver::solve(Sequence &sequence, map<string, float> params) //inherited virtual
{
    params.emplace("debugLevel", 1); //set the default debug info level
    params.emplace("useHoGdet", 1); //set the default debug info level
    params.emplace("useCSdet", 0); //set the default debug info level
    params.emplace("useSURFdet", 0); //set the default debug info level
    params.emplace("maxPartCandidates", 300); //set the default debug info level

    int maxPartCandidates = params.at("maxPartCandidates");
    bool useHoG = params.at("useHoGdet");
    int debugLevel = params.at("debugLevel");

    sequence.computeInterpolation(params); //interpolate the sequences

	//first slice up the sequences
	if(debugLevel>=1)
		cout << "TLPSSolver started, slicing sequence..." << endl;

    vector<vector<Frame*> > slices = slice(sequence.getFrames());

	if(debugLevel>=1)
		cout << slices.size() << " sequence slices created." << endl;

	vector<vector<Solvlet> > sequenceSolvlets; //one vector of solvlets per slice

	if(debugLevel>=1)
		cout << "Solving slices..." << endl;
	for(uint32_t sliceNumber=0; sliceNumber<slices.size(); ++sliceNumber)
    {		//for every slice, build a factor grph
		if(debugLevel>=1)
			cout << "Interpolating slice "<< sliceNumber << endl;
        vector<Frame*> seqSlice = slices[sliceNumber]; //the slice we are working with, interpolated

		//train detectors
        Detector * testDet;
        if(useHoG)
            testDet = new HogDetector();
		ColorHistDetector chDetector;
		vector<Frame*> trainingFrames;
		trainingFrames.push_back(seqSlice.front()); //set training frame by index
		trainingFrames.push_back(seqSlice.back());
		
        //chDetector.train(trainingFrames, params);
        testDet->train(trainingFrames, params);

		vector<vector<vector<LimbLabel> > > detections; //numbers of labels per part, per frame, for this slice

		//initialise first level
        for (uint32_t i=0; i<seqSlice.size(); ++i) // access by reference, the type of i is int&
        {
            //init detections to contain all detects for the sequence slice
        	detections.push_back(vector<vector<LimbLabel> >());
        }

		//first do the detections, and store them

		for(uint32_t currentFrame=1; currentFrame<seqSlice.size()-1; ++currentFrame) //for every frame but first and last
		{
			//current frame represents the current frame, previous frame is currentFrame-1, next frame is currentFrame+1

            cerr << "Detecting on frame " << seqSlice[currentFrame]->getID() << endl;

            vector<vector<LimbLabel> > labels, temp;
            labels = testDet->detect(seqSlice[currentFrame], params, labels); //detect labels based on keyframe training

            for(uint32_t currentSize=0; currentSize<maxPartCandidates && currentSize<labels.size(); ++currentSize)
                temp.push_back(labels[currentSize]);

            detections[currentFrame] = temp; //store all detections into detections
		}

		vector<size_t> numbersOfLabels; //numbers of labels per part

        //the first an last frames of detections are always empty

		for(uint32_t i=0; i<detections.size(); ++i)
		{
			for(uint32_t j=0; j<detections[i].size(); ++j)
			{
				numbersOfLabels.push_back(detections[i][j].size());
			}
		} //numbers of labels now contains the numbers of labels, and their respective variations

		//use this to shape the space, this will be rather large
		Space space(numbersOfLabels.begin(), numbersOfLabels.end());
		Model gm(space);

        //OK

        cerr << "Computing slice factors..." << endl;

        int suppFactors=0, jointFactors=0, anchorFactors=0, tempFactors=0;
		//now do the factors
		for(uint32_t currentFrame=1; currentFrame<seqSlice.size()-1; ++currentFrame) //for every frame but first and last
		{
			tree<BodyPart> partTree = seqSlice[currentFrame]->getSkeleton().getPartTree();
			tree<BodyPart>::iterator partIter, parentPartIter;

			//construct the image score cost factors
			//label score cost
            cerr << "Computing Factors at Frame " << seqSlice[currentFrame]->getID() << endl;
			vector<vector<LimbLabel> > labels = detections[currentFrame];
			for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter) //for each of the detected parts
			{
				vector<int> varIndices; //create vector of indices of variables
                int varID = (currentFrame-1)*partTree.size()+partIter->getPartID();
                varIndices.push_back(varID); //push first value in

				size_t scoreCostShape[]={labels[partIter->getPartID()].size()}; //number of labels
				
				ExplicitFunction<float> scoreCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

				for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
				{
                    float cost = computeScoreCost(labels[partIter->getPartID()].at(i), params); //compute the label score cost
                    if(cost==0)
                        cost = FLT_MIN;
                    scoreCostFunc(i) = cost;
				}

				Model::FunctionIdentifier scoreFid = gm.addFunction(scoreCostFunc); //explicit function add to graphical model
				gm.addFactor(scoreFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                suppFactors++;

                if(currentFrame == 1) //anchor to first skeleton in sequence
                {
                    //we already know the shape from the previous functions
                    ExplicitFunction<float> anchorCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

                    for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        float cost = computeAnchorCost(labels[partIter->getPartID()].at(i), seqSlice[0], params);
                        if(cost==0)
                            cost = FLT_MIN;
                        anchorCostFunc(i) = cost;
                    }

                    Model::FunctionIdentifier anchorFid = gm.addFunction(anchorCostFunc); //explicit function add to graphical model
                    gm.addFactor(anchorFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    anchorFactors++;
                }

                //anchor to last frame of the slice (i.e. to frame=seqSlice.size()-1
                else if(currentFrame==(seqSlice.size()-2)) //anchor to
                {
                    //we already know the shape from the previous functions
                    ExplicitFunction<float> anchorCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

                    for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        float cost = computeAnchorCost(labels[partIter->getPartID()].at(i), seqSlice[seqSlice.size()-1], params);
                        if(cost==0)
                            cost = FLT_MIN;
                        anchorCostFunc(i) = cost;
                    }

                    Model::FunctionIdentifier anchorFid = gm.addFunction(anchorCostFunc); //explicit function add to graphical model
                    gm.addFactor(anchorFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    anchorFactors++;
                }

                if(partIter!=partTree.begin()) //if iterator is not on root node, there is always a parent body part
                {
                    //this factor needs to be in reverse order, for sorting purposes
                    varIndices.clear();
                    parentPartIter=partTree.parent(partIter); //find the parent of this part
                    varIndices.push_back((currentFrame-1)*partTree.size()+parentPartIter->getPartID()); //push back parent partID as the second variable index
                    varIndices.push_back(varID); //push first value in (parent, this)

                    size_t jointCostShape[]={labels[parentPartIter->getPartID()].size(), labels[partIter->getPartID()].size()}; //number of labels
                    ExplicitFunction<float> jointCostFunc(jointCostShape, jointCostShape+2); //explicit function declare

                    for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        for(uint32_t j=0; j<labels[parentPartIter->getPartID()].size(); ++j)
                        {
                            //for every child/parent pair, compute score
                            float cost = computeJointCost(labels[partIter->getPartID()].at(i), labels[parentPartIter->getPartID()].at(j), params);
                            if(cost==0)
                                cost = FLT_MIN;
                            jointCostFunc(j, i) = cost;
                        }
                    }

                    Model::FunctionIdentifier jointFid = gm.addFunction(jointCostFunc); //explicit function add to graphical model
                    gm.addFactor(jointFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    jointFactors++;
                }
				
                //futre temporal link (if not anchored)
                if(currentFrame<seqSlice.size()-2)
                {
                    varIndices.clear();
                    varIndices.push_back((currentFrame-1)*partTree.size()+partIter->getPartID()); //push back parent partID as the second variable index
                    varIndices.push_back((currentFrame)*partTree.size()+partIter->getPartID());

                    size_t futureCostShape[]={labels[partIter->getPartID()].size(), detections[currentFrame+1][partIter->getPartID()].size()}; //number of labels
                    ExplicitFunction<float> futureTempCostFunc(futureCostShape, futureCostShape+2); //explicit function declare

                    for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        for(uint32_t j=0; j<detections[currentFrame+1][partIter->getPartID()].size(); ++j)
                        {
                            float cost = computeFutureTempCost(labels[partIter->getPartID()].at(i), detections[currentFrame+1][partIter->getPartID()].at(j), params);
                            if(cost==0)
                                cost = FLT_MIN;
                            futureTempCostFunc(i,j) = cost;
                        }
                    }
                    Model::FunctionIdentifier futureTempCostFid = gm.addFunction(futureTempCostFunc); //explicit function add to graphical model
                    gm.addFactor(futureTempCostFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    tempFactors++;
                }
			}
		}

        if(debugLevel>=1)
        {
            float n,k;
            n=seqSlice.size()-2; //num non-anchor frames
            k=seqSlice.front()->getSkeleton().getPartTree().size(); //number of bones
            float expectedSuppFactors=n*k; //n*k
            float expectedJointFactors=n*(k-1); //n*(k-1)
            float expectedAnchorFactors=2*k; //2*k
            float expectedTempFactors=(n-1)*k; //(n-1)*k

            assert(expectedSuppFactors==suppFactors);
            assert(expectedJointFactors==jointFactors);
            assert(expectedAnchorFactors==anchorFactors);
            assert(expectedTempFactors==tempFactors);
        }

        cerr << "Solving slice factor graph" << endl;

		//now solve the slice

        if(debugLevel>=2)
            opengm::hdf5::save(gm, "gm.h5", "tlps-gm");

		const size_t maxNumberOfIterations = 100;
	   	const double convergenceBound = 1e-7;
	   	const double damping = 0.5;
	   	BeliefPropagation::Parameter parameter(maxNumberOfIterations, convergenceBound, damping);
	   	BeliefPropagation bp(gm, parameter);
		
	   	// optimize (approximately)
		BeliefPropagation::VerboseVisitorType visitor;
		bp.infer(visitor);
		//pass to solve function

		// obtain the (approximate) argmin
		vector<size_t> labeling((detections.size()-2)*detections[1].size()); //number of non-anchor frames * number of bodyparts
		bp.arg(labeling);

        vector<vector<LimbLabel> > solutionLabels;
        for(uint32_t i=0; i<seqSlice.size(); ++i)
        {
            solutionLabels.push_back(vector<LimbLabel>());
            // for(uint32_t j=0; j<detections[i].size(); ++j)
            // {
            // 	solutionLabels[i].push_back(
            // }
        }

        for(uint32_t i=1; i<seqSlice.size()-1;++i) //frames
        {
            for(uint32_t j=0; j<detections[i].size(); ++j) //parts
            {
                int solveId=(i-1)*detections[1].size()+j; //where to find the solution?
                solutionLabels[i].push_back(detections[i][j][labeling[solveId]]); //pupulate solution vector
            }
        }

		vector<Solvlet> solvlets;
		//now set up a solvlet for every frame
		for(uint32_t i=1; i<seqSlice.size();++i)
		{
			solvlets.push_back(Solvlet(seqSlice[i]->getID(), solutionLabels[i]));
		}

		sequenceSolvlets.push_back(solvlets);
	}


	if(debugLevel>=1)
		cout << sequenceSolvlets.size() << " slices solved." << endl;
	//sequence solvlets now contain all the solvlets for the entire sequence

	//rearrange this into one vector of solvlets before returning
	vector<Solvlet> retSolve;
    for(uint32_t i=0; i<sequenceSolvlets.size();++i)
	{
        for(uint32_t j=0; j<sequenceSolvlets[i].size(); ++j)
		{
			retSolve.push_back(sequenceSolvlets[i][j]);
		}
	}

	return retSolve;
}

float TLPSSolver::evaluateSolution(Frame* frame, vector<LimbLabel> labels, map<string, float> params)
{
	//Solution evaluator - how should this work? What should it return? What should it do?
	return 0;
}

int TLPSSolver::findFrameIndexById(int id, vector<Frame*> frames)
{
	for(uint32_t i=0; i<frames.size(); ++i)
	{
		if(frames[i]->getID()==id)
			return i;
	}
	return -1;
}

float TLPSSolver::computeScoreCost(const LimbLabel& label, map<string, float> params)
{
	//@FIX
	params.emplace("imageCoeff", 0.5);
	params.emplace("scoreIndex", 0);
	float lambda = params.at("imageCoeff");
	float scoreIndex = params.at("scoreIndex");
	//for now, just return the first available score
	vector<Score> scores = label.getScores();
	if(scores.size()>0)
	{
		return lambda*scores[scoreIndex].getScore();
	}
	else //if no scores present return -1
		return -1;
}
    
float TLPSSolver::computeJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params)
{
	//emplace default
	params.emplace("jointCoeff", 0.5);
	float lambda = params.at("jointCoeff");
	//@PARAM there should be a parameter that sets the joint connectivity penalty constant (alpha)
	//compute the joint joining cost for the skeleton
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	child.getEndpoints(c0,c1);
	parent.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return lambda*(pow((c0.x-p1.x), 2)+pow((c0.y-p1.y), 2));
}

float TLPSSolver::computePriorCost(const LimbLabel& label, const BodyPart& prior, Skeleton& skeleton, map<string, float> params)
{
	//emplace default
	params.emplace("priorCoeff", 0.5);
	float lambda = params.at("priorCoeff");
	//@PARAM there should be a parameter that sets the strength of the penalty when deviating from the prior
	//compute the cost to interpolation prior
	Point2f p0, p1;
	label.getEndpoints(p0,p1);
	Point2f pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
	Point2f pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

	//return the sum of squared distances between the corresponding joints, prior to label
	return lambda*(pow((p0.x-pp0.x), 2)+pow((p0.y-pp0.y), 2)+pow((p1.x-pp1.x), 2)+pow((p1.y-pp1.y), 2));
}
    
float TLPSSolver::computePastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, map<string, float> params)
{
	//emplace default
	params.emplace("tempCoeff", 0.5);
	float lambda = params.at("tempCoeff");
	//compute the temporal connection cost to label in the past

	//@PARAM there should be a parameter that sets the temporal cost constant (beta)
	//compute the joint joining cost for the skeleton
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	thisLabel.getEndpoints(c0,c1);
	pastLabel.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return lambda*(pow((c0.x-p0.x), 2)+pow((c0.y-p0.y), 2)+pow((c1.x-p1.x), 2)+pow((c1.y-p1.y), 2));
}

float TLPSSolver::computeFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, map<string, float> params)
{
	//emplace default
	params.emplace("tempCoeff", 0.5);
	float lambda = params.at("tempCoeff");
	//compute temporal connection cost to label in the future
	//@PARAM there needs to be a beta param here as well
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	thisLabel.getEndpoints(c0,c1);
	futureLabel.getEndpoints(p0,p1);

    float score = lambda*(pow((c0.x-p0.x), 2)+pow((c0.y-p0.y), 2)+pow((c1.x-p1.x), 2)+pow((c1.y-p1.y), 2));
	//return the squared distance from the lower parent joint p1, to the upper child joint c0
    if(score==0)
        cerr<<"Zero Score!" << endl;
    return score;
}

float TLPSSolver::computeAnchorCost(const LimbLabel& thisLabel, Frame* anchor, map<string, float> params)
{
	//emploace default
	params.emplace("anchorCoeff", 1.0);
	float lambda = params.at("anchorCoeff");
	
	int limbId = thisLabel.getLimbID();

	vector<Point2f> partPolygon = anchor->getPartPolygon(limbId);
	vector<Point2f> labelPolygon = thisLabel.getPolygon();

	assert(partPolygon.size()==labelPolygon.size());

	//return the error between these'
	float score=0;
    for(size_t i=0; i<partPolygon.size(); ++i)
		score+= pow(partPolygon[i].x-labelPolygon[i].x, 2)+pow(partPolygon[i].y-labelPolygon[i].y,2);
	
	return lambda*score;
	//compute the cost of anchoring this label 
}

vector<vector<Frame*> > TLPSSolver::slice(const vector<Frame*>& frames) //separate the sequence into slices, for temporal solve
{
	//frames should be sliced into frame sets, where every non Keyframe non Lockframe frame should belong to a BOUNDED set
	//unbounded sets are not included in the solve
	vector<vector<Frame*> > aux;
	vector<vector<Frame*> > slices;

	vector<Frame*> currentSet;
	//bool isOpen;
	for(uint32_t i=0; i<frames.size(); ++i)
	{
		currentSet.push_back(frames[i]); //push the frame to current set
		if(frames[i]->getFrametype()==KEYFRAME || frames[i]->getFrametype()==LOCKFRAME)
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
	for(uint32_t i=0;i<aux.size(); ++i)
	{
		if(aux[i].at(0)->getFrametype()==LOCKFRAME || aux[i].at(0)->getFrametype()==KEYFRAME) //if the set STARTS with a keyframe or a lockframe
		{
			if(aux[i].back()->getFrametype()==LOCKFRAME || aux[i].back()->getFrametype()==KEYFRAME) //if the set ENDS with a keyframe or a lockframe
			{
				if(aux[i].size()>2) //if size is greater than two elements
					slices.push_back(aux[i]); //push back slice
			}
		}
	}

	return slices; 
}

