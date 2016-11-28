#include "skeleton.hpp"
//See Skeleton.hpp for more info
namespace SPEL
{
  //default constructor
  Skeleton::Skeleton(void) 
  {
    /// name of the specific instance of
    m_name = "Uninitialized";
    /// tree of bodyparts is component of the body model
    /// scale factor, used for scaling
    m_scale = 1.0f;
  }

  Skeleton::Skeleton(const Skeleton &skeleton) 
    : m_name(skeleton.m_name),
    m_partTree(skeleton.m_partTree),
    m_jointTree(skeleton.m_jointTree),
    m_scale(skeleton.m_scale)
  {
  }

  Skeleton::Skeleton(Skeleton && skeleton) 
    : m_name(std::move(skeleton.m_name)),
    m_partTree(std::move(skeleton.m_partTree)),
    m_jointTree(std::move(skeleton.m_jointTree)),
    m_scale(std::move(skeleton.m_scale))
  {
  }

  Skeleton::~Skeleton(void) 
  {
  }

  Skeleton &Skeleton::operator=(const Skeleton &skeleton) 
  {
    if (this == &skeleton)
      return *this;

    m_name = skeleton.m_name;
    m_partTree = skeleton.m_partTree;
    m_jointTree = skeleton.m_jointTree;
    m_scale = skeleton.m_scale;
    return *this;
  }

  Skeleton & Skeleton::operator=(Skeleton && skeleton) 
  {
    std::swap(m_name, skeleton.m_name);
    std::swap(m_partTree, skeleton.m_partTree);
    std::swap(m_jointTree, skeleton.m_jointTree);
    m_scale = std::move(skeleton.m_scale);

    return *this;
  }

  bool Skeleton::operator==(const Skeleton &skeleton) const 
  {
    return std::equal(m_partTree.begin(), m_partTree.end(), 
      skeleton.m_partTree.begin(), skeleton.m_partTree.end());
  }

  bool Skeleton::operator!=(const Skeleton &skeleton) const 
  {
    return !(*this == skeleton);
  }

  std::string Skeleton::getName(void) const 
  {
    return m_name;
  }

  void Skeleton::setName(const std::string &name) 
  {
    m_name = name;
  }

  tree <BodyPart> Skeleton::getPartTree(void) const 
  {
    return m_partTree;
  }

  tree<BodyPart>* Skeleton::getPartTreePtr(void) 
  {
    return &m_partTree;
  }

  void Skeleton::setPartTree(const tree <BodyPart> &partTree) 
  {
    m_partTree = partTree;
  }

  tree <BodyJoint> Skeleton::getJointTree(void) const 
  {
    return m_jointTree;
  }

  tree<BodyJoint>* Skeleton::getJointTreePtr(void) 
  {
    return &m_jointTree;
  }

  void Skeleton::setJointTree(const tree <BodyJoint> &jointTree) 
  {
    m_jointTree = jointTree;
  }

  float Skeleton::getScale(void) const 
  {
    return m_scale;
  }

  void Skeleton::setScale(const float scale) 
  {
    m_scale = scale;
  }

  uint32_t Skeleton::getPartTreeCount(void) const 
  {
    return static_cast<uint32_t>(m_partTree.size());
  }

  BodyJoint *Skeleton::getBodyJoint(const int jointID) const 
  {
    for (auto &joint : m_jointTree)
      if (joint.getLimbID() == jointID)
        return &joint;

    return nullptr;
  }

  BodyPart* Skeleton::getBodyPart(const int partID) const 
  {
    for (auto &part : m_partTree)
      if (part.getPartID() == partID)
        return &part;

    return nullptr;
  }

  void Skeleton::infer2D(void) 
  {
    for (auto &joint : m_jointTree)
      joint.setImageLocation(joint.getImageLocation() * m_scale);
  }

  void Skeleton::infer3D(void)
  {
    if (m_scale == 0.0f)
    {
      const auto &str = "Scale shouldn't be equal zero.";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }

    std::map <uint32_t, float> dz;

    for (const auto& tree : m_partTree)
    {
      const auto len3d = tree.getRelativeLength();
      const auto len2d = sqrt(spelHelper::distSquared(
        getBodyJoint(tree.getParentJoint())->getImageLocation(), 
        getBodyJoint(tree.getChildJoint())->getImageLocation()));
      //compute the difference, this must be the depth
      const auto diff = pow(len3d, 2) - pow(len2d / m_scale, 2);
      if (diff < 0)
        dz[tree.getPartID()] = 0;
      else if (sqrt(diff) > len3d)
        dz[tree.getPartID()] = len3d;
      else
        dz[tree.getPartID()] = sqrt(diff);
    }

    for (auto &tree : m_partTree)
    {
      const auto child = getBodyJoint(tree.getChildJoint());
      const auto parent = getBodyJoint(tree.getParentJoint());
      if (tree.getPartID() == 0) //if zero partID, we are on the root part
      {
        parent->setSpaceLocation(cv::Point3f(parent->getImageLocation().x / 
          m_scale, parent->getImageLocation().y / m_scale, 0));
      }
      const auto sign = child->getDepthSign() == 0 ? -1.0f : 1.0f;
      const auto z = tree == *(m_partTree.begin()) ? 0.0f : 
        parent->getSpaceLocation().z;
      child->setSpaceLocation(cv::Point3f(child->getImageLocation().x / 
        m_scale, child->getImageLocation().y / m_scale, sign * 
        dz.at(tree.getPartID()) + z));
    }
  }

}
