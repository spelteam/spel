#include "skeleton.hpp"
//See Skeleton.hpp for more info
namespace SPEL
{
  //default constructor
  Skeleton::Skeleton(void) noexcept
  {
    /// name of the specific instance of
    m_name = "Uninitialized";
    /// tree of bodyparts is component of the body model
    /// scale factor, used for scaling
    m_scale = 1.0f;
  }

  Skeleton::Skeleton(const Skeleton &skeleton) noexcept
    : m_name(skeleton.m_name),
    m_partTree(skeleton.m_partTree),
    m_jointTree(skeleton.m_jointTree),
    m_scale(skeleton.m_scale)
  {
  }

  Skeleton::Skeleton(Skeleton && skeleton) noexcept
    : m_name(std::move(skeleton.m_name)),
    m_partTree(std::move(skeleton.m_partTree)),
    m_jointTree(std::move(skeleton.m_jointTree)),
    m_scale(std::move(skeleton.m_scale))
  {
  }

  Skeleton::~Skeleton(void) noexcept
  {
  }

  Skeleton &Skeleton::operator=(const Skeleton &skeleton) noexcept
  {
    if (this == &skeleton)
      return *this;

    m_name = skeleton.m_name;
    m_partTree = skeleton.m_partTree;
    m_jointTree = skeleton.m_jointTree;
    m_scale = skeleton.m_scale;
    return *this;
  }

  Skeleton & Skeleton::operator=(Skeleton && skeleton) noexcept
  {
    std::swap(m_name, skeleton.m_name);
    std::swap(m_partTree, skeleton.m_partTree);
    std::swap(m_jointTree, skeleton.m_jointTree);
    m_scale = std::move(skeleton.m_scale);

    return *this;
  }

  bool Skeleton::operator==(const Skeleton &skeleton) const noexcept
  {
    auto count = m_partTree.size();
    if (count != skeleton.m_partTree.size())
      return false;
    return std::equal(m_partTree.begin(), m_partTree.end(), 
      skeleton.m_partTree.begin());
  }

  bool Skeleton::operator!=(const Skeleton &skeleton) const noexcept
  {
    return !(*this == skeleton);
  }

  std::string Skeleton::getName(void) const noexcept
  {
    return m_name;
  }

  void Skeleton::setName(const std::string &name) noexcept
  {
    m_name = name;
  }

  tree <BodyPart> Skeleton::getPartTree(void) const noexcept
  {
    return m_partTree;
  }

  tree<BodyPart>* Skeleton::getPartTreePtr(void) noexcept
  {
    return &m_partTree;
  }

  void Skeleton::setPartTree(const tree <BodyPart> &partTree) noexcept
  {
    m_partTree = partTree;
  }

  tree <BodyJoint> Skeleton::getJointTree(void) const noexcept
  {
    return m_jointTree;
  }

  tree<BodyJoint>* Skeleton::getJointTreePtr(void) noexcept
  {
    return &m_jointTree;
  }

  void Skeleton::setJointTree(const tree <BodyJoint> &jointTree) noexcept
  {
    m_jointTree = jointTree;
  }

  float Skeleton::getScale(void) const noexcept
  {
    return m_scale;
  }

  void Skeleton::setScale(const float scale) noexcept
  {
    m_scale = scale;
  }

  uint32_t Skeleton::getPartTreeCount(void) const noexcept
  {
    return static_cast<uint32_t>(m_partTree.size());
  }

  BodyJoint *Skeleton::getBodyJoint(const int jointID) const noexcept
  {
    for (auto &joint : m_jointTree)
      if (joint.getLimbID() == jointID)
        return &joint;

    return nullptr;
  }

  BodyPart* Skeleton::getBodyPart(const int partID) const noexcept
  {
    for (auto &part : m_partTree)
      if (part.getPartID() == partID)
        return &part;

    return nullptr;
  }

  void Skeleton::infer2D(void) noexcept
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
