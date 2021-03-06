// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
#include "sequence.hpp"

namespace SPEL
{
  Sequence::Sequence(void)
  {

  }

  Sequence::Sequence(const Sequence& seq)
  {
    id = seq.getID();
    name = seq.getName();
    for (auto f : seq.getFrames())
      frames.push_back(f->clone(new Frame()));
  }

  Sequence::Sequence(int idx, std::string seqName, std::vector<Frame*> seq)
  {
    id = idx;
    name = seqName;
    for (auto f : seq)
      frames.push_back(f->clone(new Frame()));
  }

  Sequence::~Sequence(void)
  {
    for (auto f : frames)
      delete f;
    frames.clear();
  }

  std::string Sequence::getName() const
  {
    return name;
  }

  void Sequence::setName(const std::string& _name)
  {
    name = _name;
  }

  int Sequence::getID() const
  {
    return id;
  }

  void Sequence::setID(const int& _id)
  {
    id = _id;
  }

  std::vector<Frame*> Sequence::getFrames() const
  {
    std::vector<Frame*> frames_;

    for (auto f : frames)
      frames_.push_back(f->clone(new Frame()));
    return frames_;
  }

  Frame* Sequence::getFrame(uint32_t id) const
  {
      Frame* frame=frames.at(id);
      return frame->clone(new Frame());

  }

  void Sequence::setFrames(const std::vector<Frame *> &_frames)
  {
    for (auto f : frames)
      delete f;
    frames.clear();

    for (auto f : _frames)
      frames.push_back(f->clone(new Frame()));
  }

  void Sequence::computeInterpolation(std::map<std::string, float> &params)
  {
    params.emplace("useDefaultScale", 1);
    params.emplace("defaultScale", 180);
    params.emplace("interpolate2d", 0);
    bool interpolate2d = params.at("interpolate2d");

    float defaultScale = params.at("defaultScale");
    float useDefaultScale = params.at("useDefaultScale");

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
        //infer keyframe/lockframe 3D locations
        Skeleton skel = frames[i]->getSkeleton();
        if (useDefaultScale)
        {
          skel.setScale(defaultScale);
        }
        skel.infer3D();
        frames[i]->setSkeleton(skel);

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
      if (aux[i].front()->getFrametype() == LOCKFRAME || aux[i].front()->getFrametype() == KEYFRAME) //if the set STARTS with a keyframe or a lockframe
      {
        if (aux[i].back()->getFrametype() == LOCKFRAME || aux[i].back()->getFrametype() == KEYFRAME) //if the set ENDS with a keyframe or a lockframe
        {
          if (aux[i].size()>2) //if size is greater than two elements
            slices.push_back(aux[i]); //push back slice
        }
      }
    }

    for (uint32_t i = 0; i < slices.size(); ++i)
    {
      if (!interpolate2d)
        interpolateSlice(slices[i], params);
      else
        interpolateSlice2D(slices[i], params);
    }
  }

  std::vector<Frame*> Sequence::interpolateSlice2D(std::vector<Frame*> slice, std::map<std::string, float> params)
  {
    params.emplace("debugLevel", 1);
    int debugLevel = params.at("debugLevel");

    //check that the slice contains a keyframe/lockframe at each end
    assert(slice.front()->getFrametype() == LOCKFRAME || slice.front()->getFrametype() == KEYFRAME);
    assert(slice.back()->getFrametype() == LOCKFRAME || slice.back()->getFrametype() == KEYFRAME);

    for (uint32_t i = 1; i < slice.size() - 1; ++i)
    {
      assert(slice[i]->getFrametype() != KEYFRAME && slice[i]->getFrametype() != LOCKFRAME); //if the inbetween frames are not keyframes and lockframes
    }

    Skeleton prevSkel = slice.front()->getSkeleton();
    Skeleton futureSkel = slice.back()->getSkeleton();
    tree<BodyJoint> prevJointTree, futureJointTree;

    prevJointTree = prevSkel.getJointTree();
    futureJointTree = futureSkel.getJointTree();

    std::vector<cv::Point2f> prevJoints;
    std::vector<cv::Point2f> futureJoints;

    for (uint32_t i = 0; i < prevJointTree.size(); ++i)
    {
      prevJoints.push_back(cv::Point2f());
      futureJoints.push_back(cv::Point2f());
    }

    //set prevJoints
    for (tree<BodyJoint>::iterator jt = prevJointTree.begin(); jt != prevJointTree.end(); ++jt)
    {
      prevJoints[jt->getLimbID()] = jt->getImageLocation();
    }

    //set futureJoints
    for (tree<BodyJoint>::iterator jt = futureJointTree.begin(); jt != futureJointTree.end(); ++jt)
    {
      futureJoints[jt->getLimbID()] = jt->getImageLocation();
    }

    //compute interpolation
    for (uint32_t i = 1; i < slice.size() - 1; ++i)
    {
      Skeleton interpolatedSkeleton = prevSkel;
      tree<BodyJoint> jointTree = interpolatedSkeleton.getJointTree();
      for (tree<BodyJoint>::iterator jt = jointTree.begin(); jt != jointTree.end(); ++jt)
      {
        jt->setImageLocation(prevJoints[jt->getLimbID()] * 0.5 + futureJoints[jt->getLimbID()] * 0.5);
      }
      interpolatedSkeleton.setJointTree(jointTree);

      slice[i]->setSkeleton(interpolatedSkeleton);

      //frame type should be updated to interpolaion
      Interpolation interpolatedFrame;
      interpolatedSkeleton.infer3D(); //infer 3D from the interpolated 2D joints
      interpolatedFrame.setSkeleton(interpolatedSkeleton);
      interpolatedFrame.setID(slice[i]->getID());
      interpolatedFrame.SetMaskFromPath(slice[i]->GetMaskPath());
      interpolatedFrame.setGroundPoint(slice[i]->getGroundPoint());
      interpolatedFrame.SetImageFromPath(slice[i]->GetImagePath());

      //delete slice[i];
      *slice[i] = interpolatedFrame;
    }
    return slice;
  }

  std::vector<Frame*> Sequence::interpolateSlice(std::vector<Frame*> slice, std::map<std::string, float> params)
  {
    //first make sure that the front and back frames HAVE 3D space locations computed
    //if not, compute them
    params.emplace("debugLevel", 1);
    int debugLevel = params.at("debugLevel");

    //check that the slice contains a keyframe/lockframe at each end
    assert(slice.front()->getFrametype() == LOCKFRAME || slice.front()->getFrametype() == KEYFRAME);
    assert(slice.back()->getFrametype() == LOCKFRAME || slice.back()->getFrametype() == KEYFRAME);

    for (uint32_t i = 1; i < slice.size() - 1; ++i)
    {
      assert(slice[i]->getFrametype() != KEYFRAME && slice[i]->getFrametype() != LOCKFRAME); //if the inbetween frames are not keyframes and lockframes
    }

    Skeleton prevSkel = slice.front()->getSkeleton();
    Skeleton futureSkel = slice.back()->getSkeleton();

    //set up the two part trees, the part tree is used for structure
    tree<BodyPart> partTree = slice.front()->getSkeleton().getPartTree();
    tree<BodyPart>::iterator partIter, parentIter; //the body part iterator

    std::vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Vector4f> > rotationsPast; //unrotate past vector by this
    std::vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Vector4f> > rotationsFuture; //unrotate future vector by this

    for (uint32_t i = 0; i < partTree.size(); ++i)
    {
      rotationsPast.push_back(Eigen::Quaternionf::Identity());
      rotationsFuture.push_back(Eigen::Quaternionf::Identity());
    }

    if (debugLevel >= 3)
      std::cerr << "\t Setup complete" << std::endl;

    for (partIter = partTree.begin(); partIter != partTree.end(); ++partIter)
    {
      //get partent and child joints of previous part
      BodyJoint* childJointP = prevSkel.getBodyJoint(partIter->getChildJoint());
      BodyJoint* parentJointP = prevSkel.getBodyJoint(partIter->getParentJoint());

      //get parent and child joints of future part
      BodyJoint* childJointF = futureSkel.getBodyJoint(partIter->getChildJoint());
      BodyJoint* parentJointF = futureSkel.getBodyJoint(partIter->getParentJoint());

      //convert Point3f to Vector3f
      cv::Point3f childLocP = childJointP->getSpaceLocation();
      cv::Point3f parentLocP = parentJointP->getSpaceLocation();

      cv::Point3f childLocF = childJointF->getSpaceLocation();
      cv::Point3f parentLocF = parentJointF->getSpaceLocation();

      Eigen::Vector3f prevVec(childLocP.x - parentLocP.x, childLocP.y - parentLocP.y, childLocP.z - parentLocP.z);
      Eigen::Vector3f futureVec(childLocF.x - parentLocF.x, childLocF.y - parentLocF.y, childLocF.z - parentLocF.z);
      
      //get all rotations that need to happen
      Eigen::Vector3f prevDvec(1.0, 0, 0), futureDvec(1.0, 0, 0);
      parentIter = partTree.parent(partIter);
      std::vector<int> path;
      while (parentIter != NULL)
      {
        path.push_back(parentIter->getPartID());
        parentIter = partTree.parent(parentIter);
      }

      for (std::vector<int>::reverse_iterator pathIter = path.rbegin(); pathIter != path.rend(); ++pathIter)
      {
        prevVec = rotationsPast[*pathIter].inverse()._transformVector(prevVec);
        futureVec = rotationsFuture[*pathIter].inverse()._transformVector(futureVec);
      }

      //now compute the quaternions
      Eigen::Quaternionf prevRot, futureRot;
      prevRot = prevRot.FromTwoVectors(prevDvec, prevVec.normalized());
      futureRot = prevRot.FromTwoVectors(futureDvec, futureVec.normalized());

      rotationsPast[partIter->getPartID()] = prevRot.normalized();
      rotationsFuture[partIter->getPartID()] = futureRot.normalized();
    }

    if (debugLevel >= 3)
      std::cerr << "\t Quaternions computed" << std::endl;

    //now that quaterion rotations are computed, we can compute the new skeleton
    for (uint32_t i = 1; i < slice.size() - 1; ++i)
    {
      float time = (float)i / (slice.size() - 1);
      //only the t value depends on frame number
      Skeleton interpolatedSkeleton = prevSkel;

      std::vector<Eigen::Vector3f> currentPartState;
      partTree = interpolatedSkeleton.getPartTree();

      for (uint32_t j = 0; j < partTree.size(); ++j)
      {
        currentPartState.push_back(Eigen::Vector3f());
      }

      for (partIter = partTree.begin(); partIter != partTree.end(); ++partIter)
      {
        Eigen::Vector3f partVec(1.0, 0, 0);
        //partVec = partVec*partIter->getRelativeLength();

        parentIter = partIter;
        std::vector<int> path;
        while (parentIter != NULL)
        {
          path.push_back(parentIter->getPartID());
          parentIter = partTree.parent(parentIter);
        }

        for (std::vector<int>::iterator pathIter = path.begin(); pathIter != path.end(); ++pathIter)
        {
          Eigen::Quaternionf qRes, qStart, qEnd;
          qStart = rotationsPast[*pathIter];
          qEnd = rotationsFuture[*pathIter];

          qRes = qStart.slerp(time, qEnd);

          partVec = qRes._transformVector(partVec);
        }

        //partVec = rotationsPast[partIter->getPartID()]._transformVector(partVec);
        currentPartState[partIter->getPartID()] = partVec;
      }
      //now update skeleton accordingly - locations just need to be added together (i.e. add child joint to vector)

      if (debugLevel >= 3)
        std::cerr << "\t New joint locations computed" << std::endl;

      for (partIter = partTree.begin(); partIter != partTree.end(); ++partIter)
      {
        if (partIter->getPartID() == 0) //if this is the root bone
        {   //first set up the interpolated location of the root joint
          BodyJoint* prevRootJoint = prevSkel.getBodyJoint(partIter->getParentJoint()); //this should be the previous root joint
          BodyJoint* futureRootJoint = futureSkel.getBodyJoint(partIter->getParentJoint()); //this should be the future root joint
          cv::Point3f prevRootLoc = prevRootJoint->getSpaceLocation();
          cv::Point3f futureRootLoc = futureRootJoint->getSpaceLocation();

          float X = spelHelper::interpolateValue(prevRootLoc.x, futureRootLoc.x, i, slice.size() - 1);
          float Y = spelHelper::interpolateValue(prevRootLoc.y, futureRootLoc.y, i, slice.size() - 1);
          float Z = spelHelper::interpolateValue(prevRootLoc.z, futureRootLoc.z, i, slice.size() - 1);

          BodyJoint* currentRootJoint = interpolatedSkeleton.getBodyJoint(partIter->getParentJoint()); //this should be the future root joint
          currentRootJoint->setSpaceLocation(cv::Point3f(X, Y, Z));
        }
        BodyJoint* parentJointT = interpolatedSkeleton.getBodyJoint(partIter->getParentJoint());
        BodyJoint* childJointT = interpolatedSkeleton.getBodyJoint(partIter->getChildJoint());

        //root location needs to be interpolated

        Eigen::Vector3f partState = currentPartState[partIter->getPartID()] * partIter->getRelativeLength();;

        //parent and child joints now contain the correct location information
        cv::Point3f p = parentJointT->getSpaceLocation();

        childJointT->setSpaceLocation(cv::Point3f(p.x + partState.x(), p.y + partState.y(), p.z + partState.z()));

        //parentJointT->setSpaceLocation(Point3f(parentJoint.x(), parentJoint.y(), parentJoint.z()));
      }

      if (debugLevel >= 3)
        std::cerr << "\t Skeleton generated" << std::endl;

      //now skeleton should be set for this frame
      //slice[i].setSkeleton(interpolatedSkeleton);

      //frame type should be updated to interpolaion
      Interpolation interpolatedFrame;
      interpolatedSkeleton.infer2D(); //infer 2D from the interpolated 3D joints
      interpolatedFrame.setSkeleton(interpolatedSkeleton);
      interpolatedFrame.setID(slice[i]->getID());
      interpolatedFrame.SetMaskFromPath(slice[i]->GetMaskPath());
      interpolatedFrame.setGroundPoint(slice[i]->getGroundPoint());
      interpolatedFrame.SetImageFromPath(slice[i]->GetImagePath());

      //delete slice[i];
      *slice[i] = interpolatedFrame;
    }

    //cout << "Interpolated keyframes" << endl;
    return slice; //result contains Frame*'s that have been modified according to

  }

  void Sequence::estimateUniformScale(std::map<std::string, float> &params)
  {
    assert(frames.size() != 0);
    //the purpose of this function is to set identical scale for all skeletons in the sequence
    //it will disable the use default scale parameter
    params.emplace("useDefaultScale", 0);
    params.at("useDefaultScale") = 0;
    //now go through every KEYFRAME skeleton

    std::vector<float> scales;
    for (uint32_t i = 0; i < frames.size(); ++i)
    {
      if (frames[i]->getFrametype() == KEYFRAME)
      {
        Skeleton skel = frames[i]->getSkeleton();
        tree<BodyPart> partTree = skel.getPartTree();
        for (tree <BodyPart>::iterator tree = partTree.begin(); tree != partTree.end(); ++tree)
        {
          float len3d = tree->getRelativeLength();
          float len2d = sqrt(spelHelper::distSquared(skel.getBodyJoint(tree->getParentJoint())->getImageLocation(), skel.getBodyJoint(tree->getChildJoint())->getImageLocation()));
          scales.push_back(len2d / len3d); //compute the difference, this must be the depth
        }
      }
    }
    float scale = *std::max_element(scales.begin(), scales.end());

    for (uint32_t i = 0; i < frames.size(); ++i)
    {
      Skeleton skel(frames[i]->getSkeleton());
      skel.setScale(scale);
      frames[i]->setSkeleton(skel);
    }
  }

}
