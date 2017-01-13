// SPEL definitions
#include "predef.hpp"

#include "bodyjointhandler.h"

#include <QDomElement>
#include <QDomDocument>
#include "projectattr.h"

namespace posegui {
  using namespace SPEL;
  BodyJoint BodyJointHandler::read(const QDomElement &data){
    BodyJoint bodyJoint;
    const QDomNamedNodeMap& attrs = data.attributes();

    int id = attrs.namedItem(BodyJointAttrs::ID)
      .nodeValue().toInt();
    Point2f imgLocation;
    imgLocation.x = attrs.namedItem(BodyJointAttrs::X)
      .nodeValue().toFloat();
    imgLocation.y = attrs.namedItem(BodyJointAttrs::Y)
      .nodeValue().toFloat();
    QString depthSign = attrs.namedItem(BodyJointAttrs::DEPTH)
      .nodeValue();

    bodyJoint.setLimbID(id);
    bodyJoint.setDepthSign(depthSign == "true" || depthSign == "1");
    bodyJoint.setImageLocation(imgLocation);
    return bodyJoint;
  }

  QDomElement BodyJointHandler::write(const BodyJoint &model, QDomDocument &controller){
    QDomElement elem = controller
      .createElement(BodyJointAttrs::JOINT_TAG);
    elem.setAttribute(BodyJointAttrs::ID, model.getLimbID());
    elem.setAttribute(BodyJointAttrs::X, model.getImageLocation().x);
    elem.setAttribute(BodyJointAttrs::Y, model.getImageLocation().y);
    elem.setAttribute(BodyJointAttrs::DEPTH, model.getDepthSign());
    return elem;
  }

}

//PUBLIC

