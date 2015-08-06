#ifndef BODYJOINTHANDLER_H
#define BODYJOINTHANDLER_H

#include "modelhandler.h"
#include <bodyJoint.hpp>

class QDomElement;
class QDomDocument;

namespace posegui {
  using namespace SPEL;
  class BodyJointHandler : public ModelHandler < SPEL::BodyJoint, QDomElement, QDomDocument >
  {
  public:
    SPEL::BodyJoint read(const QDomElement &data) override;
    QDomElement write(const SPEL::BodyJoint &model, QDomDocument &controller) override;
    virtual ~BodyJointHandler() override{}
  };

}

#endif // BODYJOINTHANDLER_H
