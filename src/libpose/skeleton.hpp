#ifndef _LIBPOSE_SKELETON_HPP_
#define _LIBPOSE_SKELETON_HPP_

#include <string>
#include <tree.hh>
#include "bodyPart.hpp"

using namespace std;

class Skeleton
{
  public:
    string toString(void);
    void learnDepth(Skeleton &skel);
    void infer2D(void);
    Skeleton & operator=(const Skeleton &s);
    bool operator==(const Skeleton &s) const;
    bool operator!=(const Skeleton &s) const;
  private:
    string name;
    tree <BodyPart> partTree;
    tree <BodyJoint> jointTree;
    float scale;
};

#endif  // _LIBPOSE_SKELETON_HPP_

