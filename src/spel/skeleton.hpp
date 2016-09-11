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
  /// <summary>
  /// Represents human body model.
  /// </summary>  
  class Skeleton
  {
  public:
    /// <summary>
    /// Initializes a new instance of the <see cref="Skeleton"/> class.
    /// </summary>
    Skeleton(void) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="Skeleton"/> class.
    /// Copy constructor.
    /// </summary>
    /// <param name="skeleton">The skeleton.</param>
    Skeleton(const Skeleton &skeleton) noexcept;
    /// <summary>
    /// Initializes a new instance of the <see cref="Skeleton"/> class.
    /// Move constructor.
    /// </summary>
    /// <param name="skeleton">The skeleton.</param>
    Skeleton(Skeleton &&skeleton) noexcept;
    /// <summary>
    /// Finalizes an instance of the <see cref="Skeleton"/> class.
    /// </summary>
    ~Skeleton(void) noexcept;
    /// <summary>
    /// Recalculates the 2D coordinates of each <see cref="BodyJoint"/>.
    /// </summary>
    void infer2D(void) noexcept;
    /// <summary>
    /// Recalculates the 3D coordinates of each <see cref="BodyJoint"/>.
    /// </summary>
    void infer3D(void);
    /// <summary>Assignment operator for the specified skeleton.</summary>
    /// <param name="skeleton">The skeleton.</param>
    /// <returns>The skeleton.</returns>
    Skeleton & operator=(const Skeleton &skeleton) noexcept;
    /// <summary>Move operators for the specified skeleton.</summary>
    /// <param name="skeleton">The skeleton.</param>
    /// <returns>The skeleton.</returns>
    Skeleton& operator=(Skeleton &&skeleton) noexcept;
    /// <summary>Comparison operator.</summary>
    /// <param name="skeleton">The skeleton.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator==(const Skeleton &skeleton) const noexcept;
    /// <summary>Comparison operator.</summary>
    /// <param name="skeleton">The skeleton.</param>
    /// <returns>The result of the comparison.</returns>
    bool operator!=(const Skeleton &skeleton) const noexcept;
    /// <summary>Gets the skeleton name.</summary>
    /// <returns>The skeleton name.</returns>
    std::string getName(void) const noexcept;
    /// <summary>Sets the seketon name.</summary>
    /// <param name="name">The skeleton name.</param>
    void setName(const std::string &name) noexcept;
    /// <summary>Gets the part tree.</summary>
    /// <returns>The part tree.</returns>
    tree <BodyPart> getPartTree(void) const noexcept;
    /// <summary>Gets the part tree.</summary>
    /// <returnsThe part tree></returns>
    tree<BodyPart>* getPartTreePtr(void) noexcept;
    /// <summary>Sets the part tree.</summary>
    /// <param name="partTree">The part tree.</param>
    void setPartTree(const tree <BodyPart> &partTree) noexcept;
    /// <summary>Gets the joint tree.</summary>
    /// <returns>the joint tree.</returns>
    tree <BodyJoint> getJointTree(void) const noexcept;
    /// <summary>Gets the joint tree.</summary>
    /// <returns>the joint tree.</returns>
    tree<BodyJoint>* getJointTreePtr(void) noexcept;
    /// <summary>Sets the joint tree.</summary>
    /// <param name="jointTree">The joint tree.</param>
    void setJointTree(const tree <BodyJoint> &jointTree) noexcept;
    /// <summary>Gets the scale.</summary>
    /// <returns>The scale.</returns>
    float getScale(void) const noexcept;
    /// <summary>Sets the scale.</summary>
    /// <param name="scale">The scale.</param>
    void setScale(const float scale) noexcept;
    /// <summary>Gets the part count in the tree.</summary>
    /// <returns></returns>
    uint32_t getPartTreeCount(void) const noexcept;
    /// <summary>Gets the body joint.</summary>
    /// <param name="jointID">The joint identifier.</param>
    /// <returns>The body joint.</returns>
    BodyJoint* getBodyJoint(const int jointID) const noexcept;
    /// <summary>Gets the body part.</summary>
    /// <param name="partID">The part identifier.</param>
    /// <returns>The body part.</returns>
    BodyPart* getBodyPart(const int partID) const noexcept;
  private:    
    /// <summary>The skeleton name</summary>
    std::string m_name;    
    /// <summary>The tree of body parts.</summary>
    tree <BodyPart> m_partTree;    
    /// <summary>The tree of body joints.</summary>
    tree <BodyJoint> m_jointTree;    
    /// <summary>The scale factor.</summary>
    float m_scale;
  };

}

#endif  // _LIBPOSE_SKELETON_HPP_
