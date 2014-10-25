#ifndef _LIBPOSE_SKELETON_HPP_
#define _LIBPOSE_SKELETON_HPP_

#include <string>
#include <tree.hh>
#include "bodyPart.hpp"

using namespace std;

class Skeleton
{
  public:
    Skeleton(void);
//TODO (Vitaliy Koshura): Need implementation
    string toString(void);
//TODO (Vitaliy Koshura): Need implementation
    void learnDepth(Skeleton &skel);
//TODO (Vitaliy Koshura): Need implementation
    void infer2D(void);
    Skeleton & operator=(const Skeleton &s);
    bool operator==(const Skeleton &s) const;
    bool operator!=(const Skeleton &s) const;
    string getName(void) const;
    void setName(string _name);
    tree <BodyPart> getPartTree(void) const;
    void setPartTree(tree <BodyPart> _partTree);
    tree <BodyJoint> getJointTree(void) const;
    void setJointTree(tree <BodyJoint> _jointTree);
//TODO (Vitaliy Koshura): Need unit test
    BodyJoint* getBodyJoint(int jointID) const;
    float getScale(void) const;
    void setScale(float _scale);
    uint32_t getPartTreeCount(void) const;
  private:
    string name;
    tree <BodyPart> partTree;
    tree <BodyJoint> jointTree;
    float scale;
};

#endif  // _LIBPOSE_SKELETON_HPP_

