#ifndef _LIBPOSE_SKELETON_HPP_
#define _LIBPOSE_SKELETON_HPP_

#include <string>
#include <tree.hh>
#include "bodyPart.hpp"

using namespace std;

//Human body model
class Skeleton
{
  public:
    Skeleton(void);
//TODO (Vitaliy Koshura): Need implementation
    string toString(void); // must return Skeleton as string
//TODO (Vitaliy Koshura): Need implementation
    void learnDepth(Skeleton &skel);
//TODO (Vitaliy Koshura): Need implementation
    void infer2D(void);
// All these functions just give access to the object fields
    Skeleton & operator=(const Skeleton &s);
    bool operator==(const Skeleton &s) const;
    bool operator!=(const Skeleton &s) const;
    string getName(void) const;
    void setName(string _name);
    tree <BodyPart> getPartTree(void) const; 
    void setPartTree(tree <BodyPart> _partTree); 
    tree <BodyJoint> getJointTree(void) const; 
    void setJointTree(tree <BodyJoint> _jointTree);
    float getScale(void) const;
    void setScale(float _scale);

    uint32_t getPartTreeCount(void) const; // count of bodypart elements, included in the tree
    BodyJoint* getBodyJoint(int jointID) const; // search a joint by id and return a pointer to its address
  private:
    string name; // name of the specific instance of
    tree <BodyPart> partTree; // tree of bodyparts is component of the body model
    tree <BodyJoint> jointTree; // tree of joints is component of the body model
    float scale; // scale factor, used for scaling
};

#endif  // _LIBPOSE_SKELETON_HPP_

