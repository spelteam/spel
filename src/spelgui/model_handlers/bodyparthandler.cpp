// SPEL definitions
#include "predef.hpp"

#include "bodyparthandler.h"

#include <QDomElement>
#include <QDomDocument>
#include "projectattr.h"

namespace posegui {

  BodyPart BodyPartHandler::read(const QDomElement &data){
    BodyPart bodyPart;
    const QDomNamedNodeMap& attrs = data.attributes();


    int id = attrs.namedItem(BodyPartAttrs::ID)
      .nodeValue().toInt();
    QString isOccluded = attrs.namedItem(BodyPartAttrs::IS_OCCLUDE)
      .nodeValue();

    bodyPart.setPartID(id);
    bodyPart.setIsOccluded(isOccluded == "true" || isOccluded == "1");

    return bodyPart;
  }

  QDomElement BodyPartHandler::write(const BodyPart &model, QDomDocument &controller){
    QDomElement elem = controller
      .createElement(BodyPartAttrs::PART_TAG);
    elem.setAttribute(BodyPartAttrs::ID, model.getPartID());
    elem.setAttribute(BodyPartAttrs::IS_OCCLUDE, model.getIsOccluded());
    return elem;
  }

}

