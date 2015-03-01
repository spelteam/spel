#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include "poseHelper.hpp"
#include "interpolation.hpp"

#include "sequence.hpp"

using namespace Eigen;

Sequence::Sequence(void)
{

}

Sequence::Sequence(const Sequence& seq)
{
    this->name = seq.name;
    this->frames.clear();
    this->frames = seq.frames;
}

Sequence::Sequence(int idx, string seqName, vector<Frame*> seq)
{
    this->id = idx;
    name=this->name;
    frames.clear();
    for(uint32_t i=0; i<seq.size(); ++i)
    {
        seq[i]->setID(i);
        frames.push_back(seq[i]);
    }
}

string Sequence::getName() const
{
    return this->name;
}

void Sequence::setName(const string& _name)
{
    this->name=_name;
}

int Sequence::getID() const
{
    return this->id;
}

void Sequence::setID(const int& _id)
{
    this->id = _id;
}

vector<Frame*> Sequence::getFrames() const
{
    return this->frames;
}

void Sequence::setFrames(const vector<Frame *> _frames)
{
//    for(int i=0; i<this->frames.size();++i)
//    {
//        delete frames[i];
//    }
    this->frames.clear();
    this->frames=_frames;
}

void Sequence::computeInterpolation(map<string, float> &params)
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

    for(uint32_t i=0; i<slices.size(); ++i)
    {
        vector<Frame*> slice = interpolateSlice(slices[i], params);
        slices[i] = slice;
    }

    //now store the results
    for(uint32_t i=0; i<slices.size();++i)
    {
        vector<Frame*> slice = slices[i];
        for(uint32_t j=0; j<slice.size();++j)
        {
            if(slice[j]->getFrametype()==INTERPOLATIONFRAME) //only replace interpolations, everything keyframes and lockframes are fine
            {
//                delete frames[slice[j]->getID()];
                *(frames[slice[j]->getID()])=*(slice[j]); //copy the value from the resulting slice pointer
                //delete slice[j]; //delete the old slice pointer to free memory
            }
            else
            {
                Skeleton skel = slice[j]->getSkeleton();
                skel.infer3D();
                slice[j]->setSkeleton(skel);
            }

        }
    }
}

vector<Frame*> Sequence::interpolateSlice(vector<Frame*> slice, map<string, float> params)
{
   //first make sure that the front and back frames HAVE 3D space locations computed
   //if not, compute them

   //check that the slice contains a keyframe/lockframe at each end
   assert(slice.front()->getFrametype()==LOCKFRAME || slice.front()->getFrametype()==KEYFRAME);
   assert(slice.back()->getFrametype()==LOCKFRAME || slice.back()->getFrametype()==KEYFRAME);

   for(uint32_t i=1; i<slice.size()-1; ++i)
   {
        assert(slice[i]->getFrametype()!=KEYFRAME && slice[i]->getFrametype()!=LOCKFRAME); //if the inbetween frames are not keyframes and lockframes
   }

   params.emplace("useDefaultScale", 1);
   params.emplace("defaultScale", 120);

   float defaultScale =  params.at("defaultScale");
   float useDefaultScale = params.at("useDefaultScale");

   vector<Frame*> result;

   for(uint32_t i=0; i<slice.size();++i)
   {
       Frame* ptr=NULL;
       result.push_back(ptr);
   } //fill vector with empty pointers


   //attach the start and end, and generate everything in between in this function
   result[0] = slice[0];
   result[result.size()-1] = slice[slice.size()-1];

   //front and back are parent skeletons, their information

   //matrix.create(3, 3, DataType<float>::type);
   //now we need to build a tree of AxisAngles that reflects the rotations
   Skeleton prevSkel = slice.front()->getSkeleton();
   Skeleton futureSkel = slice.back()->getSkeleton();
   if(useDefaultScale)
   {
       prevSkel.setScale(defaultScale);
       futureSkel.setScale(defaultScale);
   }
   prevSkel.infer3D();
   futureSkel.infer3D();

   //set up the two part trees, the part tree is used for structure
   tree<BodyPart> partTree = slice.front()->getSkeleton().getPartTree();
   tree<BodyPart>::iterator partIter, parentIter; //the body part iterator

   vector<Eigen::Quaternionf> rotations;
   vector<Eigen::Vector3f> unrotatedPast;
   vector<Eigen::Vector3f> unrotatedFuture;

   for(uint32_t i=0; i<partTree.size(); ++i)
   {
       rotations.push_back(Quaternionf::Identity());
       unrotatedPast.push_back(Vector3f());
       unrotatedFuture.push_back(Vector3f());
       //unrotatedNodes.push_back(Eigen::Vector3f());
   }

   cerr << "\t Setup complete" << endl;

   for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
   {
//        cerr << "\t\t Setting up rotations for part " << partIter->getPartID() << endl;
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
       parentIter = partTree.parent(partIter);
       vector<Eigen::Quaternionf> previousNodes;
       while(parentIter!=NULL)
       {
           previousNodes.push_back(rotations[parentIter->getPartID()]);
           parentIter = partTree.parent(parentIter);
       }


       //now unrotate starting from root down
       //cerr << previousNodes.size() << endl;
       for(int i=previousNodes.size()-1; i>=0; --i)
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

   cerr << "\t Quaternions computed" << endl;

   //now that quaterion rotations are computed, we can compute the new skeleton
   for(uint32_t i=1; i<slice.size()-1; ++i)
   {
       //only the t value depends on frame number
       Skeleton interpolatedSkeleton = prevSkel;
       if(useDefaultScale)
           interpolatedSkeleton.setScale(defaultScale);
       vector<Vector3f> currentPartState;
       partTree = 	interpolatedSkeleton.getPartTree();

       for(uint32_t j=0; j<partTree.size(); ++j)
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

           angle = PoseHelper::interpolateFloat(0, angle, i, slice.size()); //interpolate the angle

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
           parentIter = partTree.parent(partIter);
           while(parentIter!=NULL)
           {
               //if there is a parent rotate by its quaternion
               Eigen::Quaternionf prevQuat = rotations[parentIter->getPartID()];

               float angleP = 2*acos(prevQuat.w());

               float xp = prevQuat.x()/sqrt(1.0-prevQuat.w()*prevQuat.w());
               float yp = prevQuat.y()/sqrt(1.0-prevQuat.w()*prevQuat.w());
               float zp = prevQuat.z()/sqrt(1.0-prevQuat.w()*prevQuat.w());

               Vector3f prevAxis(xp,yp,zp);

               angleP = PoseHelper::interpolateFloat(0, angleP, i, slice.size()); //interpolate the angle

               prevQuat = AngleAxisf(angleP, prevAxis);

               prevVec = prevQuat._transformVector(prevVec);

               parentIter = partTree.parent(parentIter);
           }

           currentPartState[partIter->getPartID()] = prevVec;
       }
       //now update skeleton accordingly - locations just need to be added together (i.e. add child joint to vector)

           cerr << "\t New joint locations computed" << endl;

       for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
       {
           Vector3f childJoint = currentPartState[partIter->getPartID()];
           Point3f parentJointP  = prevSkel.getBodyJoint(0)->getSpaceLocation();
           Vector3f parentJoint(parentJointP.x, parentJointP.y, parentJointP.z);
           //Vector3f parentJoint(0,0,0); //this is the root location of the prevSkel

           parentIter = partTree.parent(partIter);
           while(parentIter!=NULL && parentIter!=partTree.begin())
           {
               //if there is a parent, first unrotate by its quaternion
               childJoint=childJoint+currentPartState[parentIter->getPartID()];
               parentJoint=parentJoint+currentPartState[parentIter->getPartID()];
               parentIter = partTree.parent(parentIter);
           }

           //parent and child joints now contain the correct location information
           BodyJoint* childJointT = interpolatedSkeleton.getBodyJoint(partIter->getChildJoint());
           BodyJoint* parentJointT = interpolatedSkeleton.getBodyJoint(partIter->getParentJoint());

           childJointT->setSpaceLocation(Point3f(childJoint.x(), childJoint.y(), childJoint.z()));
           parentJointT->setSpaceLocation(Point3f(parentJoint.x(), parentJoint.y(), parentJoint.z()));
       }

       cerr << "\t Skeleton generated" << endl;

       //now skeleton should be set for this frame
       //slice[i].setSkeleton(interpolatedSkeleton);

       //frame type should be updated to interpolaion
       Interpolation *interpolatedFrame = new Interpolation();
       interpolatedSkeleton.infer2D(); //infer 2D from the interpolated 3D joints
       interpolatedFrame->setSkeleton(interpolatedSkeleton);
       interpolatedFrame->setID(slice[i]->getID());
       interpolatedFrame->setMask(slice[i]->getMask());
       interpolatedFrame->setGroundPoint(slice[i]->getGroundPoint());
       interpolatedFrame->setImage(slice[i]->getImage());

       //delete slice[i];
       result[i] = interpolatedFrame;
   }

   //cout << "Interpolated keyframes" << endl;
   return result; //result contains Frame*'s that have been modified according to

}

void Sequence::estimateUniformScale(map<string,float> &params)
{
    assert(frames.size()!=0);
    //the purpose of this function is to set identical scale for all skeletons in the sequence
    //it will disable the use default scale parameter
    params.emplace("useDefaultScale", 0);
    params.at("useDefaultScale") = 0;
    //now go through every KEYFRAME skeleton

    vector<float> scales;
    for(uint32_t i=0; i<frames.size();++i)
    {

        if(frames[i]->getFrametype()==KEYFRAME)
        {
            Skeleton skel = frames[i]->getSkeleton();
            tree<BodyPart> partTree = skel.getPartTree();
            for (tree <BodyPart>::iterator tree = partTree.begin(); tree != partTree.end(); ++tree)
            {
              float len3d = tree->getRelativeLength();
              float len2d = sqrt(PoseHelper::distSquared(skel.getBodyJoint(tree->getParentJoint())->getImageLocation(), skel.getBodyJoint(tree->getChildJoint())->getImageLocation()));
              scales.push_back(len2d/len3d); //compute the difference, this must be the depth
            }
        }
    }
    float scale = *std::max_element(scales.begin(), scales.end());

    for(uint32_t i=0; i<frames.size();++i)
    {
        Skeleton skel(frames[i]->getSkeleton());
        skel.setScale(scale);
        frames[i]->setSkeleton(skel);
    }
}
