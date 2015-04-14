#ifndef _LIBPOSE_SKELETON_HPP_
#define _LIBPOSE_SKELETON_HPP_

#include <string>
#include <tree.hh>
#include "bodyPart.hpp"

using namespace std;

///Human body model
class Skeleton
{
  public:
    Skeleton(void);
//TODO (Vitaliy Koshura): Need implementation
    string toString(void); // must return Skeleton as string
//TODO (Vitaliy Koshura): Need implementation
    void learnDepth(Skeleton &skel);
    void infer2D(void);
    void infer3D(void);
// All these functions just give access to the object fields
    Skeleton & operator=(const Skeleton &s);
    bool operator==(const Skeleton &s) const;
    bool operator!=(const Skeleton &s) const;
    string getName(void) const;
    void setName(string _name);
    tree <BodyPart> getPartTree(void) const; 
    ///direct access
    tree<BodyPart>* getPartTreePtr();
    void setPartTree(tree <BodyPart> _partTree); 
    tree <BodyJoint> getJointTree(void) const; 
    ///direct access
    tree<BodyJoint>* getJointTreePtr();
    void setJointTree(tree <BodyJoint> _jointTree);
    float getScale(void) const;
    void setScale(float _scale);
    //void shift(Point2f point);

    /// count of bodypart elements, included in the tree
    uint32_t getPartTreeCount(void) const;
    /// search a joint by id and return a pointer to its address
    BodyJoint* getBodyJoint(int jointID) const;
    /// search a body part by id and return a pointer to its address
    BodyPart* getBodyPart( int partID ) const;
  private:
    /// name of the specific instance of
    string name;
    /// tree of bodyparts is component of the body model
    tree <BodyPart> partTree;
    /// tree of joints is component of the body model
    tree <BodyJoint> jointTree;
    /// scale factor, used for scaling
    float scale;
};

#endif  // _LIBPOSE_SKELETON_HPP_

