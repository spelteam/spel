#ifndef BODYJOINTHEADERHANDLER_H
#define BODYJOINTHEADERHANDLER_H

#include "modelhandler.h"
#include <bodyJoint.hpp>

class QDomElement;
class QDomDocument;

namespace posegui {
  using namespace SPEL;
  class BodyJointHeaderHandler : public ModelHandler < BodyJoint, QDomElement, QDomDocument >
  {
  public:
    BodyJoint read(const QDomElement &data) override;
    QDomElement write(const BodyJoint &model, QDomDocument &controller) override;
    virtual ~BodyJointHeaderHandler() override{}
  };

}
#endif // BODYJOINTHEADERHANDLER_H
