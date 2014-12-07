#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "tlpssolver.hpp"
#include "lockframe.hpp"
#include "colorHistDetector.hpp"
#include "math.h"
#include "interpolation.hpp"

using namespace Eigen;

TLPSSolver::TLPSSolver()
{
	id=0;
	name="TLPS";
}

TLPSSolver::~TLPSSolver()
{

}

Solution TLPSSolver::solve(const vector<Frame*>& frames) //inherited virtual
{
	map<string, float> params; //set the default parameters vector

	//set some parameter deefaults
	params.at("scoreIndex")=0;
	
	//pass to next level solve function, for ISM computing
	return this->solve(frames, params);
}

Solution TLPSSolver::solve(const vector<Frame*>& frames, map<string, float> params) //inherited virtual
{
	Solution solution;

	//first slice up the sequences
	vector<vector<Frame*> > slices = slice(frames, params);

	for(uint sliceNumber=0; sliceNumber<slices.size(); ++sliceNumber)
	{
		//for every slice, build a factor graph
		vector<Frame*> seqSlice = slices[sliceNumber]; //the slice we are working with 

		//train detectors
		ColorHistDetector chDetector;
		vector<Frame*> trainingFrames;
		trainingFrames.push_back(seqSlice.front()); //set training frame by index
		trainingFrames.push_back(seqSlice.back());
		
		chDetector.train(trainingFrames, params);

		//no need to detect on keyframes and lockframes, use their joints to link to rather than labels
		vector<vector<LimbLabel> > pastLabels;
		for(uint currentFrame=1; currentFrame<seqSlice.size()-1; ++currentFrame) //for every frame but first and last
		{
			//current frame represents the current frame, previous frame is currentFrame-1, next frame is currentFrame+1

			vector<vector<LimbLabel> > labels = chDetector.detect(frames[currentFrame], params); //detect labels based on keyframe training

			vector<size_t> numbersOfLabels; //numbers of labels per part

			for(uint i=0; i<labels.size(); ++i)
			{
				numbersOfLabels.push_back(labels[i].size());
			} //numbers of labels now contains the numbers

			Space space(numbersOfLabels.begin(), numbersOfLabels.end());
			Model gm(space);

			tree<BodyPart> partTree = frames[currentFrame]->getSkeleton().getPartTree();
			tree<BodyPart>::iterator partIter, parentPartIter;

			//construct the image score cost factors

			//compute the prior cost (if any)

			//construct the future temporal link

			//construct the past temporal link
		}
	}

	//pass to solve function
	return solution;
}

float TLPSSolver::evaluateSolution(Frame* frame, vector<LimbLabel> labels, map<string, float> params)
{
	//Solution evaluator
	return 0;
}

int TLPSSolver::findFrameIndexById(int id, vector<Frame*> frames)
{
	for(uint i=0; i<frames.size(); ++i)
	{
		if(frames[i]->getID()==id)
			return i;
	}
	return -1;
}

float TLPSSolver::computeScoreCost(const LimbLabel& label, map<string, float> params)
{
	//@FIX
	//for now, just return the first available score
	vector<Score> scores = label.getScores();
	if(scores.size()>0)
	{
		int scoreIndexString = params.at("scoreIndex");
		return scores[scoreIndexString].getScore();
	}
	else //if no scores present return -1
		return -1;
}
    
float TLPSSolver::computeJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params)
{
	//@PARAM there should be a parameter that sets the joint connectivity penalty constant (alpha)
	//compute the joint joining cost for the skeleton
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	child.getEndpoints(c0,c1);
	parent.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return pow((c0.x-p1.x), 2)+pow((c0.y-p1.y), 2);
}

float TLPSSolver::computePriorCost(const LimbLabel& label, const BodyPart& prior, Skeleton& skeleton, map<string, float> params)
{
	//@PARAM there should be a parameter that sets the strength of the penalty when deviating from the prior
	//compute the cost to interpolation prior
	Point2f p0, p1;
	label.getEndpoints(p0,p1);
	Point2f pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
	Point2f pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

	//return the sum of squared distances between the corresponding joints, prior to label
	return pow((p0.x-pp0.x), 2)+pow((p0.y-pp0.y), 2)+pow((p1.x-pp1.x), 2)+pow((p1.y-pp1.y), 2);
}
    
float TLPSSolver::computePastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, map<string, float> params)
{
	//compute the temporal connection cost to label in the past

	//@PARAM there should be a parameter that sets the temporal cost constant (beta)
	//compute the joint joining cost for the skeleton
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	thisLabel.getEndpoints(c0,c1);
	pastLabel.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return pow((c0.x-p0.x), 2)+pow((c0.y-p0.y), 2)+pow((c1.x-p1.x), 2)+pow((c1.y-p1.y), 2);
}

float TLPSSolver::computeFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, map<string, float> params)
{
	//compute temporal connection cost to label in the future
	//@PARAM there needs to be a beta param here as well
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	thisLabel.getEndpoints(c0,c1);
	futureLabel.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return pow((c0.x-p0.x), 2)+pow((c0.y-p0.y), 2)+pow((c1.x-p1.x), 2)+pow((c1.y-p1.y), 2);
}

float TLPSSolver::computeAnchorCost(const LimbLabel& thisLabel, Frame* anchor, map<string, float> params)
{
	return 0;
	//compute the cost of anchoring this label 
}

vector<vector<Frame*> > TLPSSolver::slice(const vector<Frame*>& frames, map<string, float> params) //separate the sequence into slices, for temporal solve
{
	//frames should be sliced into frame sets, where every non Keyframe non Lockframe frame should belong to a BOUNDED set
	//unbounded sets are not included in the solve
	vector<vector<Frame*> > aux;
	vector<vector<Frame*> > slices;

	vector<Frame*> currentSet;
	//bool isOpen;
	for(uint i=0; i<frames.size(); ++i)
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
	for(uint i=0;i<aux.size(); ++i)
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

vector<Frame*> TLPSSolver::interpolateSlice(vector<Frame*> slice)//, map<string, float> params)
{
	//first make sure that the front and back frames HAVE 3D space locations computed
	//if not, compute them

	//check that the slice contains a keyframe/lockframe at each end
	assert(slice.front()->getFrametype()!=LOCKFRAME && slice.front()->getFrametype()!=KEYFRAME);
	assert(slice.back()->getFrametype()!=LOCKFRAME && slice.back()->getFrametype()!=KEYFRAME);
	
	vector<Frame*> result;

	for(uint i=0; i<slice.size();++i)
	{
		Frame* ptr=NULL;
		result.push_back(ptr);
	}
	//attach the start and end, and generate everything in between in this function
	result[0] = slice[0];
	result[result.size()-1] = slice[slice.size()-1];

	//front and back are parent skeletons, their information 

	//matrix.create(3, 3, DataType<float>::type);
	//now we need to build a tree of AxisAngles that reflects the rotations
	Skeleton prevSkel = slice.front()->getSkeleton();
	Skeleton futureSkel = slice.back()->getSkeleton();

	//set up the two part trees, the part tree is used for structure
	tree<BodyPart> partTree = slice.front()->getSkeleton().getPartTree();
	tree<BodyPart>::iterator partIter, parentIter; //the body part iterator

	vector<Eigen::Quaternionf> rotations;
	vector<Eigen::Vector3f> unrotatedPast;
	vector<Eigen::Vector3f> unrotatedFuture;

	for(uint i=0; i<partTree.size(); ++i)
	{
		rotations.push_back(Eigen::Quaternionf());
		//unrotatedNodes.push_back(Eigen::Vector3f());
	}

	for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
	{	
		//get partent and child joints of previous part
		BodyJoint* childJointP = prevSkel.getBodyJoint(partIter->getChildJoint());
		BodyJoint* parentJointP = prevSkel.getBodyJoint(partIter->getParentJoint());

		//get parent and child joints of future part
		BodyJoint* childJointF = futureSkel.getBodyJoint(partIter->getChildJoint());
		BodyJoint* parentJointF = futureSkel.getBodyJoint(partIter->getParentJoint());

		//convert Point3f to Vector3f
		Point3f childLocP = childJointP->getSpaceLocation();
		Point3f parentLocP = parentJointP->getSpaceLocation();

		Point3f childLocF = childJointF->getSpaceLocation();
		Point3f parentLocF = parentJointF->getSpaceLocation();

		Eigen::Vector3f prevVec(childLocP.x-parentLocP.x, childLocP.y-parentLocP.y, childLocP.z-parentLocP.z);
		Eigen::Vector3f futureVec(childLocF.x-parentLocF.x, childLocF.y-parentLocF.y, childLocF.z-parentLocF.z);


		//get all rotations that need to happen
		vector<Eigen::Quaternionf> previousNodes;
 		do
		{
			parentIter = partTree.parent(partIter);
			if(parentIter!=NULL)
			{
	        	previousNodes.push_back(rotations[parentIter->getPartID()]);
	        }
        } while(parentIter!=NULL);
        
        //now unrotate starting from root down
        for(uint i=previousNodes.size(); i>=0; --i)
        {
        	prevVec = previousNodes[i].conjugate()._transformVector(prevVec);
        	futureVec = previousNodes[i].conjugate()._transformVector(prevVec);
        }
		//Eigen::Vector3f xaxis(1,0,0);

		//compute quaterion between two positions
		Quaternionf rotation;
		rotation.setFromTwoVectors(prevVec, futureVec); //quaternion set
		rotations[partIter->getPartID()] = rotation.normalized(); //push to vector of rotations
		unrotatedPast[partIter->getPartID()] = prevVec;
		unrotatedFuture[partIter->getPartID()] = futureVec;
	}

	//now that quaterion rotations are computed, we can compute the new skeleton
	for(uint i=1; i<slice.size()-1; ++i)
	{
		//only the t value depends on frame number
		Skeleton interpolatedSkeleton = prevSkel;
		vector<Vector3f> currentPartState;
		partTree = 	interpolatedSkeleton.getPartTree();	

		for(uint j=0; j<partTree.size(); ++j)
		{
			currentPartState.push_back(Eigen::Vector3f());
		}

		for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
		{

			Eigen::Vector3f prevVec=unrotatedPast[partIter->getPartID()];
			//rotate by this quaternion, and every quaternion up in the tree
			Quaternionf thisRot = rotations[partIter->getPartID()];

			float angle = 2*acos(thisRot.w());

			float xa = thisRot.x()/sqrt(1.0-thisRot.w()*thisRot.w());
			float ya = thisRot.y()/sqrt(1.0-thisRot.w()*thisRot.w());
			float za = thisRot.z()/sqrt(1.0-thisRot.w()*thisRot.w());

			Vector3f thisAxis(xa,ya,za);

			angle = interpolateFloat(0, angle, i, slice.size()); //interpolate the angle

			//re-generate AngleAxis object and create quaternion

			//Quaternion to axis angle
			// 			angle = 2 * acos(qw)
			// x = qx / sqrt(1-qw*qw)
			// y = qy / sqrt(1-qw*qw)
			// z = qz / sqrt(1-qw*qw)
			//Quaternion<float> q;  q = AngleAxis<float>(angle_in_radian, axis);

			thisRot = AngleAxisf(angle,thisAxis);

			prevVec = thisRot._transformVector(prevVec); //rotate the vector by 

			//convert rotation to axis angle, interpolate angle, convert back to quaternion
						
	       	//rotate hierarchy
	        do
	        {
				parentIter = partTree.parent(partIter);
				if(parentIter!=NULL)
				{
		        	//if there is a parent rotate by its quaternion
		        	Eigen::Quaternionf prevQuat = rotations[parentIter->getPartID()];

		        	float angleP = 2*acos(prevQuat.w());

					float xp = prevQuat.x()/sqrt(1.0-prevQuat.w()*prevQuat.w());
					float yp = prevQuat.y()/sqrt(1.0-prevQuat.w()*prevQuat.w());
					float zp = prevQuat.z()/sqrt(1.0-prevQuat.w()*prevQuat.w());

					Vector3f prevAxis(xp,yp,zp);

					angleP = interpolateFloat(0, angleP, i, slice.size()); //interpolate the angle

		        	prevQuat = AngleAxisf(angleP, prevAxis);

		        	prevVec = prevQuat._transformVector(prevVec);
		        }
	        } while(parentIter!=NULL);

	        currentPartState[partIter->getPartID()] = prevVec;
		}
		//now update skeleton accordingly - locations just need to be added together (i.e. add child joint to vector) 

		for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
		{
			Vector3f childJoint = currentPartState[partIter->getPartID()];
			Vector3f parentJoint(0,0,0);

	        do
	        {
				parentIter = partTree.parent(partIter);
				if(parentIter!=NULL)
				{
		        	//if there is a parent, first unrotate by its quaternion
		        	childJoint=childJoint+currentPartState[parentIter->getPartID()];
		        	parentJoint=parentJoint+currentPartState[parentIter->getPartID()];
		        }
	        } while(parentIter!=NULL);

	        //parent and child joints now contain the 
    		BodyJoint* childJointT = interpolatedSkeleton.getBodyJoint(partIter->getChildJoint());
			BodyJoint* parentJointT = interpolatedSkeleton.getBodyJoint(partIter->getParentJoint());
			childJointT->setSpaceLocation(Point3f(childJoint.x(), childJoint.y(), childJoint.z()));
			parentJointT->setSpaceLocation(Point3f(parentJoint.x(), parentJoint.y(), parentJoint.z()));
		}

		//now skeleton should be set for this frame
		//slice[i].setSkeleton(interpolatedSkeleton);

		//frame type should be updated to interpolaion
		Interpolation interpolatedFrame;
		interpolatedFrame.setSkeleton(interpolatedSkeleton);
		interpolatedFrame.setID(slice[i]->getID());
		interpolatedFrame.setMask(slice[i]->getMask());
		interpolatedFrame.setGroundPoint(slice[i]->getGroundPoint());
		interpolatedFrame.setImage(slice[i]->getImage());

		//delete slice[i];
		result[i] = &interpolatedFrame;
	}

	return result; //result contains Frame*'s that have been modified according to 

}

inline float TLPSSolver::interpolateFloat(float prevAngle, float nextAngle, int step, int numSteps)
{
    float t;
    if(numSteps!=0)
        t = (float)step/(float)numSteps;
    else
        t=0;
    return prevAngle*(1-t)+nextAngle*t;
}

