#ifndef BODYJOINTSHANDLER_H
#define BODYJOINTSHANDLER_H

#include "modelhandler.h"
#include "bodyjointhandler.h"
#include <tree.hh>
#include <bodyJoint.hpp>

using namespace SPEL;

class QDomElement;

namespace posegui {

  class BodyJointsHandler : public ModelHandler < tree<BodyJoint>, QDomElement, QDomDocument, float, float, void >
  {
  public:
    void read(const QDomElement &data, tree<BodyJoint>& joints,
      float colsFactor, float rowsFactor);
    QDomElement write(const tree<BodyJoint> &model, QDomDocument &controller);
    virtual ~BodyJointsHandler() override{}
  private:
    BodyJointHandler jointHandler;
  };

}

#endif // BODYJOINTSHANDLER_H
