#ifndef _LIBPOSE_SKELETON_HPP_
#define _LIBPOSE_SKELETON_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>

// tree.hh
#include <tree.hh>

#include "bodyPart.hpp"

namespace SPEL
{
  ///Human body model
  class Skeleton
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="Skeleton"/> class.
    /// </summary>
    Skeleton(void) noexcept;
    Skeleton(const Skeleton &skeleton) noexcept;
    Skeleton(Skeleton &&skeleton) noexcept;
    virtual ~Skeleton(void) noexcept;
    void infer2D(void) noexcept;
    void infer3D(void);
    // All these functions just give access to the object fields
    Skeleton & operator=(const Skeleton &skeleton) noexcept;
    Skeleton& operator=(Skeleton &&skeleton) noexcept;
    bool operator==(const Skeleton &skeleton) const noexcept;
    bool operator!=(const Skeleton &skeleton) const noexcept;
    std::string getName(void) const noexcept;
    void setName(const std::string &name) noexcept;
    tree <BodyPart> getPartTree(void) const noexcept;
    ///direct access
    tree<BodyPart>* getPartTreePtr(void) noexcept;
    void setPartTree(const tree <BodyPart> &partTree) noexcept;
    tree <BodyJoint> getJointTree(void) const noexcept;
    ///direct access
    tree<BodyJoint>* getJointTreePtr(void) noexcept;
    void setJointTree(const tree <BodyJoint> &jointTree) noexcept;
    float getScale(void) const noexcept;
    void setScale(const float scale) noexcept;
    /// count of bodypart elements, included in the tree
    uint32_t getPartTreeCount(void) const noexcept;
    /// search a joint by id and return a pointer to its address
    BodyJoint* getBodyJoint(const int jointID) const noexcept;
    /// search a body part by id and return a pointer to its address
    BodyPart* getBodyPart(const int partID) const noexcept;
  private:
    /// name of the specific instance of
    std::string m_name;
    /// tree of bodyparts is component of the body model
    tree <BodyPart> m_partTree;
    /// tree of joints is component of the body model
    tree <BodyJoint> m_jointTree;
    /// scale factor, used for scaling
    float m_scale;
  };

}

#endif  // _LIBPOSE_SKELETON_HPP_
